/*
 * board_b.c  —  B 板完整逻辑
 *
 * 依赖（CubeMX 生成）:
 *   htim1   TIM1  互补 PWM，ARR=33599，DTG≥100，MOE已使能
 *   htim2   TIM2  ADC 触发，TRGO=Update，1kHz
 *   htim3   TIM3  D2 亮度 PWM，CH3，ARR=839
 *   htim6   TIM6  1ms 中断
 *   htim7   TIM7  100ms 中断
 *   hadc1   ADC1  IN10(PC0)，DMA Circular，TIM2_TRGO 触发
 *   huart1  USART1 115200，DMA RX Circular
 *   hi2c1   I2C1  SSD1306
 */

#include "board_b.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>

/* ── 外部句柄（CubeMX 在 main.c 中生成）─────────────── */
extern TIM_HandleTypeDef  htim1, htim2, htim3, htim6, htim7;
extern ADC_HandleTypeDef  hadc1;
extern UART_HandleTypeDef huart1;

/* ════════════════════════════════════════════════════════
 *  内部状态
 * ════════════════════════════════════════════════════════ */

/* —— ADC —— */
static uint16_t adc_dma_buf[ADC_BUF_SIZE]; /* DMA 循环缓冲 */
static float    vi_now      = 0.0f;         /* 当前 Vi 电压（实时均值）*/
static float    vi_max_1s   = 0.0f;         /* 过去 1s 最大值 */
static float    vi_min_1s   = 3.3f;         /* 过去 1s 最小值 */

/* —— PWM 模式 —— */
static uint8_t  pwm_mode    = MODE_FOLLOW;  /* 当前模式 */
static uint32_t tim1_ccr_frozen = PWM_CCR_50PCT; /* 失联时冻结值 */

/* —— D2 亮度 —— */
static uint8_t  d2_enable   = 0;            /* 0=灭, 1=亮（受K1长按控制）*/

/* —— K2 按键 —— */
static uint8_t  key2_last   = 0;
static uint32_t key2_press_ms = 0;
static uint8_t  key2_triggered = 0;         /* 短按事件标志 */

/* —— 通信 —— */
static uint8_t  uart_rx_buf[FRAME_LEN * 2]; /* DMA 接收缓冲（双倍容量防覆盖）*/
static uint8_t  uart_tx_buf[FRAME_LEN];

static uint32_t last_rx_tick   = 0;         /* 最后收到有效帧的 ms 时刻 */
static uint8_t  link_ok        = 0;         /* 1=通信正常 */
static uint8_t  tx_flag        = 0;         /* TIM7 置位，主循环发送 */

/* —— 对方状态（从A板收到）—— */
static uint8_t  peer_d1_on     = 0;
static uint8_t  peer_pwm_mode  = MODE_FOLLOW; /* A板视角的PWM模式（就是本板的）*/

/* —— 系统 ms 计数（TIM6 1ms 中断累加）—— */
static volatile uint32_t sys_ms = 0;

/* —— 失联期间 A 板发来的 D2 命令暂存 —— */
static uint8_t  d2_cmd_pending = D2_CMD_NONE;

/* ════════════════════════════════════════════════════════
 *  内部工具函数
 * ════════════════════════════════════════════════════════ */

/* Vi（ADC 均值）→ D2 CCR（微光~满亮）*/
static uint32_t vi_to_d2_ccr(float vi)
{
    if (vi < 0.0f) vi = 0.0f;
    if (vi > 3.3f) vi = 3.3f;
    /* 线性映射: 0V→D2_CCR_MIN_GLOW, 3.3V→D2_CCR_MAX */
    float ratio = vi / 3.3f;
    uint32_t ccr = (uint32_t)(D2_CCR_MIN_GLOW +
                   ratio * (D2_CCR_MAX - D2_CCR_MIN_GLOW));
    return ccr;
}

/* Vi → TIM1 CCR（10%~90%）*/
static uint32_t vi_to_pwm_ccr(float vi)
{
    if (vi < 0.0f) vi = 0.0f;
    if (vi > 3.3f) vi = 3.3f;
    float ratio = vi / 3.3f;
    uint32_t ccr = (uint32_t)(PWM_CCR_10PCT +
                   ratio * (PWM_CCR_90PCT - PWM_CCR_10PCT));
    return ccr;
}

/* 更新 TIM1 CCR（互补 PWM 占空比）*/
static void set_tim1_ccr(uint32_t ccr)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

/* 更新 D2 亮度 */
static void set_d2_brightness(uint32_t ccr)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
}

/* 读取 ADC DMA 缓冲，计算实时均值+1s max/min */
static void update_vi_stats(void)
{
    /* 取整个 1s 缓冲的均值、max、min */
    uint32_t sum = 0;
    uint16_t vmax = 0, vmin = 4095;
    for (uint16_t i = 0; i < ADC_BUF_SIZE; i++) {
        uint16_t v = adc_dma_buf[i];
        sum += v;
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
    }
    float avg  = (float)sum / ADC_BUF_SIZE;
    vi_now     = avg  * 3.3f / 4095.0f;
    vi_max_1s  = vmax * 3.3f / 4095.0f;
    vi_min_1s  = vmin * 3.3f / 4095.0f;
}

/* ════════════════════════════════════════════════════════
 *  显示刷新（SSD1306，8 行 page，每行 21 字符@6px）
 *
 *  布局：
 *   Page0: "B Board  [OK]" 或 "[LINK FAIL]"
 *   Page1: "MODE: FOLLOW " 或 MAX/MIN
 *   Page2: "Vi:  1.65V"
 *   Page3: "PWM: 50.0%"
 *   Page4: "D2:  ON " 或 OFF
 *   Page5: "Peer D1: ON "
 *   Page6: (空)
 *   Page7: (空)
 * ════════════════════════════════════════════════════════ */
static void display_refresh(void)
{
    char buf[22];
    SSD1306_Clear();

    /* Page 0: 标题 + 链路状态 */
    if (link_ok) {
        SSD1306_DrawString(0, 0, "B Board    [OK] ");
    } else {
        SSD1306_DrawString(0, 0, "B Board [FAIL]  ");
    }

    /* Page 1: PWM 模式 */
    const char *mode_str[] = {"FOLLOW", "MAX   ", "MIN   "};
    snprintf(buf, sizeof(buf), "MODE:%-6s", mode_str[pwm_mode]);
    SSD1306_DrawString(1, 0, buf);

    /* Page 2: Vi 实时值 */
    snprintf(buf, sizeof(buf), "Vi:  %.2fV  ", vi_now);
    SSD1306_DrawString(2, 0, buf);

    /* Page 3: TIM1 当前占空比百分比 */
    uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    float duty = (float)ccr / 33599.0f * 100.0f;
    snprintf(buf, sizeof(buf), "PWM: %.1f%%  ", duty);
    SSD1306_DrawString(3, 0, buf);

    /* Page 4: D2 状态 */
    snprintf(buf, sizeof(buf), "D2:  %s", d2_enable ? "ON " : "OFF");
    SSD1306_DrawString(4, 0, buf);

    /* Page 5: 对方(A板) D1 状态 */
    snprintf(buf, sizeof(buf), "PeerD1:%s", peer_d1_on ? "ON " : "OFF");
    SSD1306_DrawString(5, 0, buf);

    SSD1306_UpdateScreen();
}

/* ════════════════════════════════════════════════════════
 *  UART 发送心跳帧
 * ════════════════════════════════════════════════════════ */
static void uart_send_heartbeat(void)
{
    CommFrame_t f;
    Frame_Pack(&f,
               SRC_B,
               0,          /* B板没有D1，填0 */
               d2_enable,
               pwm_mode,
               D2_CMD_NONE);
    memcpy(uart_tx_buf, &f, FRAME_LEN);
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, FRAME_LEN);
}

/* ════════════════════════════════════════════════════════
 *  UART 接收帧解析
 * ════════════════════════════════════════════════════════ */
static void uart_parse_rx(uint16_t size)
{
    /* 在接收缓冲里搜索完整帧 */
    for (uint16_t i = 0; i + FRAME_LEN <= size; i++) {
        if (!Frame_Verify(&uart_rx_buf[i])) continue;
        if (uart_rx_buf[i + 1] != SRC_A)  continue; /* 只接受A板帧 */

        /* 更新链路时间戳 */
        last_rx_tick = sys_ms;
        link_ok = 1;

        /* 解析内容 */
        peer_d1_on = uart_rx_buf[i + 2];

        /* 处理 D2 控制命令（含失联期间 pending 的命令）*/
        uint8_t cmd = uart_rx_buf[i + 5];
        if (cmd == D2_CMD_TOGGLE) {
            d2_enable ^= 1;
            if (d2_enable) {
                /* 立即跟随当前Vi，无过渡 */
                set_d2_brightness(vi_to_d2_ccr(vi_now));
            } else {
                set_d2_brightness(0);
            }
        }

        /* 如果失联期间有 pending 命令也一并执行 */
        if (d2_cmd_pending == D2_CMD_TOGGLE) {
            d2_enable ^= 1;
            if (d2_enable)
                set_d2_brightness(vi_to_d2_ccr(vi_now));
            else
                set_d2_brightness(0);
            d2_cmd_pending = D2_CMD_NONE;
        }

        break; /* 找到一帧就够了 */
    }
}

/* ════════════════════════════════════════════════════════
 *  1s 窗口处理（在 TIM6 1ms 中断中每1s调用一次）
 * ════════════════════════════════════════════════════════ */
static void process_1s_window(void)
{
    update_vi_stats();

    if (!link_ok) {
        /* 失联：TIM1 CCR 保持冻结，D2 亮度保持 */
        return;
    }

    /* 根据模式更新 TIM1 CCR */
    uint32_t new_ccr;
    switch (pwm_mode) {
        case MODE_FOLLOW:
            /* 跟随模式在主循环实时更新，这里不重复处理 */
            return;
        case MODE_MAX:
            new_ccr = vi_to_pwm_ccr(vi_max_1s);
            break;
        case MODE_MIN:
            new_ccr = vi_to_pwm_ccr(vi_min_1s);
            break;
        default:
            return;
    }
    tim1_ccr_frozen = new_ccr;
    set_tim1_ccr(new_ccr);
}

/* ════════════════════════════════════════════════════════
 *  初始化
 * ════════════════════════════════════════════════════════ */
void BoardB_Init(void)
{
    /* 1. OLED */
    SSD1306_Init();

    /* 2. 启动 ADC DMA（循环模式，TIM2 触发）*/
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_BUF_SIZE);

    /* 3. 启动 TIM1 互补 PWM（初始占空比 50%）*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); /* 启动互补通道 CH1N */
    set_tim1_ccr(PWM_CCR_50PCT);

    /* 4. 启动 TIM3 D2 亮度 PWM（初始微光）*/
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    set_d2_brightness(0); /* 初始 D2 灭 */

    /* 5. 启动定时器中断 */
    HAL_TIM_Base_Start_IT(&htim6); /* 1ms  */
    HAL_TIM_Base_Start_IT(&htim7); /* 100ms */

    /* 6. 启动 UART DMA 接收（IDLE 中断方式）*/
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, sizeof(uart_rx_buf));

    /* 7. 初始显示 */
    display_refresh();
}

/* ════════════════════════════════════════════════════════
 *  主循环（在 main() 的 while(1) 里调用）
 * ════════════════════════════════════════════════════════ */
void BoardB_MainLoop(void)
{
    /* ── 链路超时检测 ── */
    if ((sys_ms - last_rx_tick) > HEARTBEAT_TIMEOUT_MS) {
        link_ok = 0;
    }

    /* ── 跟随模式：实时更新 TIM1 CCR ── */
    if (link_ok && pwm_mode == MODE_FOLLOW) {
        set_tim1_ccr(vi_to_pwm_ccr(vi_now));
    }

    /* ── D2 亮度实时跟随（仅 d2_enable=1 时）── */
    if (d2_enable) {
        set_d2_brightness(vi_to_d2_ccr(vi_now));
    }

    /* ── 心跳发送（tx_flag 由 TIM7 100ms 置位）── */
    if (tx_flag) {
        tx_flag = 0;
        uart_send_heartbeat();
    }

    /* ── K2 短按事件处理 ── */
    if (key2_triggered) {
        key2_triggered = 0;
        pwm_mode = (pwm_mode + 1) % 3;

        /* 切换后立即更新 CCR */
        if (pwm_mode == MODE_FOLLOW) {
            set_tim1_ccr(vi_to_pwm_ccr(vi_now));
        } else if (pwm_mode == MODE_MAX) {
            uint32_t c = vi_to_pwm_ccr(vi_max_1s);
            tim1_ccr_frozen = c;
            set_tim1_ccr(c);
        } else {
            uint32_t c = vi_to_pwm_ccr(vi_min_1s);
            tim1_ccr_frozen = c;
            set_tim1_ccr(c);
        }
    }
}

/* ════════════════════════════════════════════════════════
 *  TIM6 1ms 中断回调
 *  在 stm32f4xx_it.c 的 TIM6_DAC_IRQHandler 里调用：
 *    HAL_TIM_IRQHandler(&htim6);
 *  然后 HAL_TIM_PeriodElapsedCallback 会调到这里
 * ════════════════════════════════════════════════════════ */
void BoardB_TIM6_1ms_Callback(void)
{
    sys_ms++;

    /* ── 按键 K2 扫描（PA0，按下=高）── */
    uint8_t key2_now = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;

    if (key2_now && !key2_last) {
        /* 下降沿（按下）*/
        key2_press_ms = sys_ms;
    }
    if (!key2_now && key2_last) {
        /* 上升沿（松开）*/
        uint32_t dur = sys_ms - key2_press_ms;
        if (dur >= 20 && dur < 600) {
            /* 短按：20ms 消抖，600ms 以内 */
            key2_triggered = 1;
        }
        /* B板 K2 只有短按功能，无长按 */
    }
    key2_last = key2_now;

    /* ── 1s 窗口触发 ── */
    static uint16_t ms_cnt = 0;
    ms_cnt++;
    if (ms_cnt >= 1000) {
        ms_cnt = 0;
        process_1s_window();
    }
}

/* ════════════════════════════════════════════════════════
 *  TIM7 100ms 中断回调
 * ════════════════════════════════════════════════════════ */
void BoardB_TIM7_100ms_Callback(void)
{
    tx_flag = 1;         /* 通知主循环发送心跳 */
    display_refresh();   /* 刷新显示 */
}

/* ════════════════════════════════════════════════════════
 *  UART IDLE 接收完成回调
 *  在 HAL_UARTEx_RxEventCallback 里调用
 * ════════════════════════════════════════════════════════ */
void BoardB_UART_RxCallback(uint16_t size)
{
    uart_parse_rx(size);
    /* 重新启动 DMA 接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, sizeof(uart_rx_buf));
}
