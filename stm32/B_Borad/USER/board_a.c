/*
 * board_a.c  —  A 板完整逻辑
 *
 * 依赖（CubeMX 生成）:
 *   htim2   TIM2  ADC 触发，TRGO=Update，1kHz
 *   htim3   TIM3  D1 呼吸灯 PWM，CH3，ARR=839
 *   htim6   TIM6  1ms 中断（按键扫描 + 呼吸步进 + 1s窗口）
 *   htim7   TIM7  100ms 中断（显示刷新 + 心跳发送）
 *   hadc1   ADC1  IN10(PC0)，DMA Circular，TIM2_TRGO 触发
 *   huart1  USART1 115200，DMA RX Circular
 *   hi2c1   I2C1  SSD1306
 */

#include "board_a.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ── 外部句柄 ────────────────────────────────────────── */
extern TIM_HandleTypeDef  htim2, htim3, htim6, htim7;
extern ADC_HandleTypeDef  hadc1;
extern UART_HandleTypeDef huart1;

/* ════════════════════════════════════════════════════════
 *  内部状态
 * ════════════════════════════════════════════════════════ */

/* —— ADC —— */
static uint16_t adc_dma_buf[ADC_BUF_SIZE];
static float    vi_now    = 0.0f;   /* 实时均值（最近16点）*/
static float    vi_max_1s = 0.0f;   /* 1s 窗口最大值 */
static float    vi_min_1s = 3.3f;   /* 1s 窗口最小值 */
static float    vi_avg_1s = 0.0f;   /* 1s 窗口均值 */
static SigType_t sig_type = SIG_NONE;

/* —— D1 呼吸灯 —— */
static uint8_t  d1_enable    = 0;        /* 0=灭, 1=亮 */
static uint32_t breath_half  = BREATH_HALF_T2; /* 当前半周期(ms) */
static uint32_t breath_tick  = 0;        /* 当前步计数 */
static int8_t   breath_dir   = 1;        /* 1=亮起, -1=暗下 */
static uint32_t breath_ccr   = 0;        /* 当前 CCR 值 */

/* —— K1 按键 —— */
static uint8_t  key1_last       = 0;
static uint32_t key1_press_ms   = 0;
static uint8_t  key1_short_flag = 0;   /* 短按事件 */
static uint8_t  key1_long_flag  = 0;   /* 长按事件 */
static uint8_t  key1_long_fired = 0;   /* 防止长按重复触发 */

/* —— 通信 —— */
static uint8_t  uart_rx_buf[FRAME_LEN * 2];
static uint8_t  uart_tx_buf[FRAME_LEN];
static uint32_t last_rx_tick  = 0;
static uint8_t  link_ok       = 0;
static uint8_t  tx_flag       = 0;
static uint8_t  d2_cmd_send   = D2_CMD_NONE; /* 待发送的D2命令 */
static uint8_t  d2_cmd_pending= D2_CMD_NONE; /* 失联时暂存 */

/* —— 对方状态（从B板收到）—— */
static uint8_t  peer_d2_on    = 0;
static uint8_t  peer_pwm_mode = MODE_FOLLOW;

/* —— 系统计数 —— */
static volatile uint32_t sys_ms = 0;

/* ════════════════════════════════════════════════════════
 *  信号识别算法
 *
 *  判断顺序：
 *    1. range < 0.05V 且 avg < 0.1V → 无输入
 *    2. range < 0.05V 且 avg >= 0.1V → 直流
 *    3. 统计过零次数（以均值为基准，带 50mV 死区防抖）：
 *         2~6 次/秒 → 正弦波
 *         >= 10 次/秒 → 方波
 *    4. 辅助：斜率检测（方波跳变沿陡峭，正弦平滑）
 *         相邻点差值 > 200 ADC 计数为"陡沿"，>= 2 个陡沿 → 方波
 *    5. 兜底：默认正弦波
 * ════════════════════════════════════════════════════════ */
static SigType_t detect_signal_type(void)
{
    float range = vi_max_1s - vi_min_1s;

    /* ── 第一步：判断无输入 / 直流 ── */
    if (range < 0.05f) {
        return (vi_avg_1s < 0.1f) ? SIG_NONE : SIG_DC;
    }

    /* ── 第二步：统计过零次数 ──
     * 以 vi_avg_1s 为中轴，带 50mV 死区（约 62 ADC 计数）
     * 避免噪声在阈值附近反复触发 */
    uint16_t zero_cross = 0;
    uint16_t hysteresis_adc = 62;  /* 50mV → 50/3.3*4095 ≈ 62 */
    uint16_t mid_adc = (uint16_t)(vi_avg_1s / 3.3f * 4095.0f);
    uint16_t hi_thr = mid_adc + hysteresis_adc;
    uint16_t lo_thr = (mid_adc > hysteresis_adc) ? (mid_adc - hysteresis_adc) : 0;
    uint8_t above = (adc_dma_buf[0] > hi_thr) ? 1 : 0;

    for (uint16_t i = 1; i < ADC_BUF_SIZE; i++) {
        uint16_t v = adc_dma_buf[i];
        if (!above && v > hi_thr) {
            above = 1;
            zero_cross++;
        } else if (above && v < lo_thr) {
            above = 0;
            zero_cross++;
        }
    }

    /* ── 第三步：斜率辅助检测 ──
     * 方波跳变沿：1ms 内差值可达 2000+ ADC 计数
     * 正弦波：1ms 内最大差值约 8 ADC 计数（1Hz, 1V 幅值）*/
    uint16_t steep = 0;
    for (uint16_t i = 1; i < ADC_BUF_SIZE; i++) {
        int32_t d = (int32_t)adc_dma_buf[i] - (int32_t)adc_dma_buf[i - 1];
        if (d < 0) d = -d;
        if (d > 200) steep++;
    }

    /* ── 第四步：综合判断 ── */
    if (zero_cross >= 10 || steep >= 2) return SIG_SQUARE;
    if (zero_cross >= 2)                return SIG_SINE;

    /* 兜底：有幅度但不满足以上条件，默认正弦 */
    return SIG_SINE;
}

/* ════════════════════════════════════════════════════════
 *  1s 窗口统计
 * ════════════════════════════════════════════════════════ */
static void process_1s_window(void)
{
    uint32_t sum = 0;
    uint16_t vmax = 0, vmin = 4095;
    for (uint16_t i = 0; i < ADC_BUF_SIZE; i++) {
        uint16_t v = adc_dma_buf[i];
        sum += v;
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
    }
    vi_avg_1s = (float)sum / ADC_BUF_SIZE * 3.3f / 4095.0f;
    vi_max_1s = vmax * 3.3f / 4095.0f;
    vi_min_1s = vmin * 3.3f / 4095.0f;
    sig_type  = detect_signal_type();
}

/* ════════════════════════════════════════════════════════
 *  实时 Vi（最近16点均值）
 * ════════════════════════════════════════════════════════ */
static void update_vi_now(void)
{
    uint32_t tmp = 0;
    for (uint16_t i = ADC_BUF_SIZE - 16; i < ADC_BUF_SIZE; i++)
        tmp += adc_dma_buf[i];
    vi_now = (tmp / 16.0f) * 3.3f / 4095.0f;
}

/* ════════════════════════════════════════════════════════
 *  呼吸灯更新（在 TIM6 1ms 中断里调用）
 *  使用 sin 曲线，半周期 = breath_half ms
 * ════════════════════════════════════════════════════════ */
static void breath_step(void)
{
    if (!d1_enable) {
        /* 灭：CCR=0 */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        breath_tick = 0;
        breath_ccr  = 0;
        return;
    }

    breath_tick++;
    if (breath_tick >= breath_half) {
        breath_tick = 0;
        breath_dir  = -breath_dir; /* 换向 */
    }

    /* sin 曲线：0~PI 对应亮起，PI~2PI 对应暗下
     * 当前相位 = (breath_tick / breath_half) * PI（亮起方向）
     *           或 PI + ... （暗下方向）*/
    float phase;
    if (breath_dir == 1) {
        phase = (float)breath_tick / breath_half * 3.14159f;
    } else {
        phase = 3.14159f + (float)breath_tick / breath_half * 3.14159f;
    }
    float ratio = (sinf(phase - 3.14159f / 2.0f) + 1.0f) / 2.0f; /* 0~1 */
    breath_ccr = (uint32_t)(ratio * D1_CCR_MAX);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, breath_ccr);
}

/* ════════════════════════════════════════════════════════
 *  根据 B板 PWM 模式更新呼吸半周期
 * ════════════════════════════════════════════════════════ */
static void update_breath_period(uint8_t mode)
{
    switch (mode) {
        case MODE_FOLLOW: breath_half = BREATH_HALF_T1; break; /* 0.5s */
        case MODE_MAX:    breath_half = BREATH_HALF_T2; break; /* 1s   */
        case MODE_MIN:    breath_half = BREATH_HALF_T3; break; /* 2s   */
        default:          breath_half = BREATH_HALF_T2; break;
    }
}

/* ════════════════════════════════════════════════════════
 *  显示刷新
 * ════════════════════════════════════════════════════════ */
static const char* sig_name[] = {"NONE  ", "DC    ", "SINE  ", "SQUARE"};
static const char* mode_name[]= {"FOLLOW", "MAX   ", "MIN   "};

static void display_refresh(void)
{
    char buf[22];
    SSD1306_Clear();

    /* Page 0: 标题 + 链路 */
    SSD1306_DrawString(0, 0, link_ok ? "A Board    [OK] " : "A Board [FAIL]  ");

    /* Page 1: 信号类型 */
    snprintf(buf, sizeof(buf), "Sig:%s", sig_name[sig_type]);
    SSD1306_DrawString(1, 0, buf);

    /* Page 2: 实时 Vi */
    snprintf(buf, sizeof(buf), "Vi:  %.2fV  ", vi_now);
    SSD1306_DrawString(2, 0, buf);

    /* Page 3: 1s 最大值 */
    snprintf(buf, sizeof(buf), "Vmax:%.2fV  ", vi_max_1s);
    SSD1306_DrawString(3, 0, buf);

    /* Page 4: 1s 最小值 */
    snprintf(buf, sizeof(buf), "Vmin:%.2fV  ", vi_min_1s);
    SSD1306_DrawString(4, 0, buf);

    /* Page 5: 两板 LED 状态 */
    snprintf(buf, sizeof(buf), "D1:%s D2:%s",
             d1_enable ? "ON " : "OFF",
             peer_d2_on ? "ON " : "OFF");
    SSD1306_DrawString(5, 0, buf);

    /* Page 6: B板 PWM 模式 */
    snprintf(buf, sizeof(buf), "MODE:%s", mode_name[peer_pwm_mode]);
    SSD1306_DrawString(6, 0, buf);

    SSD1306_UpdateScreen();
}

/* ════════════════════════════════════════════════════════
 *  UART 发送心跳
 * ════════════════════════════════════════════════════════ */
static void uart_send_heartbeat(void)
{
    CommFrame_t f;
    Frame_Pack(&f, SRC_A, d1_enable, 0, 0, d2_cmd_send);
    /* 发送后清零命令，防止重复发送 */
    d2_cmd_send = D2_CMD_NONE;
    memcpy(uart_tx_buf, &f, FRAME_LEN);
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, FRAME_LEN);
}

/* ════════════════════════════════════════════════════════
 *  UART 接收解析
 * ════════════════════════════════════════════════════════ */
static void uart_parse_rx(uint16_t size)
{
    for (uint16_t i = 0; i + FRAME_LEN <= size; i++) {
        if (!Frame_Verify(&uart_rx_buf[i])) continue;
        if (uart_rx_buf[i + 1] != SRC_B)   continue;

        last_rx_tick = sys_ms;

        uint8_t was_linked = link_ok;
        link_ok = 1;

        peer_d2_on    = uart_rx_buf[i + 3];
        peer_pwm_mode = uart_rx_buf[i + 4];

        /* 更新呼吸周期 */
        update_breath_period(peer_pwm_mode);

        /* 复联后处理 pending 命令 */
        if (!was_linked && d2_cmd_pending == D2_CMD_TOGGLE) {
            d2_cmd_send    = D2_CMD_TOGGLE;
            d2_cmd_pending = D2_CMD_NONE;
        }

        break;
    }
}

/* ════════════════════════════════════════════════════════
 *  初始化
 * ════════════════════════════════════════════════════════ */
void BoardA_Init(void)
{
    /* OLED */
    SSD1306_Init();

    /* ADC DMA */
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_BUF_SIZE);

    /* D1 呼吸灯 PWM */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); /* 初始灭 */

    /* 定时器中断 */
    HAL_TIM_Base_Start_IT(&htim6); /* 1ms  */
    HAL_TIM_Base_Start_IT(&htim7); /* 100ms */

    /* UART DMA 接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, sizeof(uart_rx_buf));

    /* 初始显示 */
    display_refresh();
}

/* ════════════════════════════════════════════════════════
 *  主循环
 * ════════════════════════════════════════════════════════ */
void BoardA_MainLoop(void)
{
    /* 链路超时检测 */
    if ((sys_ms - last_rx_tick) > HEARTBEAT_TIMEOUT_MS) {
        link_ok = 0;
    }

    /* 实时更新 Vi */
    update_vi_now();

    /* 心跳发送 */
    if (tx_flag) {
        tx_flag = 0;

        /* 失联期间有 pending 命令，先暂存不发 */
        if (!link_ok && d2_cmd_pending == D2_CMD_TOGGLE) {
            /* 什么都不做，等复联 */
        }
        uart_send_heartbeat();
    }

    /* K1 短按：切换 D1 */
    if (key1_short_flag) {
        key1_short_flag = 0;
        d1_enable ^= 1;
        if (d1_enable) {
            /* 点亮时重置呼吸到起始，立即使用最新周期 */
            breath_tick = 0;
            breath_dir  = 1;
            update_breath_period(peer_pwm_mode);
        }
    }

    /* K1 长按：发送 D2 切换命令 */
    if (key1_long_flag) {
        key1_long_flag = 0;
        if (link_ok) {
            d2_cmd_send = D2_CMD_TOGGLE;
        } else {
            /* 失联期间暂存 */
            d2_cmd_pending = D2_CMD_TOGGLE;
        }
    }
}

/* ════════════════════════════════════════════════════════
 *  TIM6 1ms 中断回调
 * ════════════════════════════════════════════════════════ */
void BoardA_TIM6_1ms_Callback(void)
{
    sys_ms++;

    /* ── K1 按键扫描（PA0）── */
    uint8_t key1_now = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;

    if (key1_now && !key1_last) {
        /* 按下 */
        key1_press_ms  = sys_ms;
        key1_long_fired = 0;
    }

    if (key1_now && key1_last) {
        /* 持续按住：检测长按 */
        if (!key1_long_fired && (sys_ms - key1_press_ms) >= 600) {
            key1_long_flag  = 1;
            key1_long_fired = 1; /* 防重复 */
        }
    }

    if (!key1_now && key1_last) {
        /* 松开 */
        uint32_t dur = sys_ms - key1_press_ms;
        if (dur >= 20 && dur < 600) {
            key1_short_flag = 1; /* 短按 */
        }
        key1_long_fired = 0;
    }

    key1_last = key1_now;

    /* ── 呼吸灯步进 ── */
    breath_step();

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
void BoardA_TIM7_100ms_Callback(void)
{
    tx_flag = 1;
    display_refresh();
}

/* ════════════════════════════════════════════════════════
 *  UART IDLE 接收回调
 * ════════════════════════════════════════════════════════ */
void BoardA_UART_RxCallback(uint16_t size)
{
    uart_parse_rx(size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, sizeof(uart_rx_buf));
}
