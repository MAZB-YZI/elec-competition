/**
 * main.c — H_CAR 测试主入口
 *
 * 通过宏切换不同测试模式：
 *   TEST_SPEED_LOOP    定速直行（速度闭环）
 *   TEST_HEADING_LOOP  定航向直行（yaw 闭环）— 待实现
 *   TEST_LINE_TURN     直角循迹转弯 — 待实现
 *
 * 默认：TEST_SPEED_LOOP
 */

#include "ti_msp_dl_config.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "oled.h"
#include "delay.h"

/* ========== 测试模式选择 ========== */
#ifndef TEST_SPEED_LOOP
#ifndef TEST_HEADING_LOOP
#ifndef TEST_LINE_TURN
#define TEST_SPEED_LOOP  /* 默认模式 */
#endif
#endif
#endif

/* ========== 速度环参数 ========== */
#define SPEED_TARGET_CM_S   50      /* 目标速度 cm/s */
#define SPEED_KP            8.0f
#define SPEED_KI            1.0f
#define SPEED_KD            0.0f
#define SPEED_PWM_LIMIT     2500    /* PWM 限幅 */
#define SPEED_CORRECT_L     1.0f    /* 左轮修正系数 */
#define SPEED_CORRECT_R     0.95f   /* 右轮修正系数（实际比左轮快） */
#define LOOP_MS             10      /* 10ms 控制周期 */

/* ========== 全局变量 ========== */
static volatile uint32_t g_ms_ticks = 0U;
static volatile bool g_tick_flag = false;

/* ========== 定时器中断 ========== */
void SYS_TICK_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(SYS_TICK_INST)) {
    case DL_TIMER_IIDX_ZERO:
        ++g_ms_ticks;
        g_tick_flag = true;
        break;
    default:
        break;
    }
}

/* 编码器中断由 hcar_hal.c 的 GROUP1_IRQHandler 处理 */

/* ========== 辅助函数：显示带符号浮点 ========== */
static void display_signed_tenths(uint8_t x, uint8_t y, float value)
{
    int32_t scaled = (int32_t)(value * 10.0f);
    uint32_t mag;

    if (scaled < 0) {
        OLED_ShowString(x, y, "-", 12);
        mag = (uint32_t)(-scaled);
    } else {
        OLED_ShowString(x, y, "+", 12);
        mag = (uint32_t)scaled;
    }

    OLED_ShowNum(x + 8, y, mag / 10U, 3, 12);
    OLED_ShowString(x + 32, y, ".", 12);
    OLED_ShowNum(x + 40, y, mag % 10U, 1, 12);
}

static void display_signed_int16(uint8_t x, uint8_t y, int16_t value, uint8_t len)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        OLED_ShowNum(x + 8, y, (uint32_t)(-value), len, 12);
    } else {
        OLED_ShowString(x, y, "+", 12);
        OLED_ShowNum(x + 8, y, (uint32_t)value, len, 12);
    }
}

/* ================================================================
 *  TEST_SPEED_LOOP — 定速直行
 * ================================================================ */
#ifdef TEST_SPEED_LOOP

int main(void)
{
    PID_t pid_left, pid_right;
    EncoderPair speed;
    int16_t pwm_left, pwm_right;
    uint32_t last_loop_ms = 0U;
    uint32_t last_display_ms = 0U;
    uint32_t loop_count = 0U;

    /* 初始化 */
    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    OLED_Init();

    PID_Init(&pid_left, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_PWM_LIMIT);
    PID_Init(&pid_right, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_PWM_LIMIT);

    /* 启动编码器中断 */
    DL_GPIO_clearInterruptStatus(GPIOA,
        ENCODER1_A_ENC1_A_PIN | ENCODER2_A_ENC2_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER1_A_ENC1_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER2_A_ENC2_A_PIN);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /* 启动 1ms 定时器 */
    NVIC_ClearPendingIRQ(SYS_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(SYS_TICK_INST_INT_IRQN);

    /* 显示启动画面 */
    OLED_Clear();
    OLED_ShowString(0, 0, "SPEED LOOP TEST", 12);
    OLED_ShowString(0, 16, "TARGET:", 12);
    OLED_ShowNum(60, 16, SPEED_TARGET_CM_S, 3, 12);
    OLED_ShowString(88, 16, "cm/s", 12);
    OLED_Refresh();
    delay_ms(500U);

    last_loop_ms = g_ms_ticks;
    last_display_ms = g_ms_ticks;

    while (1) {
        uint32_t now = g_ms_ticks;

        /* 10ms 控制周期 */
        if ((now - last_loop_ms) >= LOOP_MS) {
            float dt = (float)(now - last_loop_ms) / 1000.0f;
            last_loop_ms = now;

            /* 更新编码器 */
            Encoder_Update(dt);
            speed = Encoder_GetSpeed();

            /* 目标速度转换单位：cm/s → m/s */
            float target_m_s = (float)SPEED_TARGET_CM_S / 100.0f;

            /* PID 计算（左右轮交换，右轮取反） */
            pwm_left = PID_Compute(&pid_left,
                (int16_t)(target_m_s * 1000),
                (int16_t)(speed.right * 1000),
                dt);
            pwm_right = PID_Compute(&pid_right,
                (int16_t)(target_m_s * 1000),
                (int16_t)(-speed.left * 1000),
                dt);

            /* 输出到电机 */
            Motor_SetLeftSpeed(pwm_left);
            Motor_SetRightSpeed(pwm_right);

            ++loop_count;
        }

        /* 200ms 刷新显示 */
        if ((now - last_display_ms) >= 200U) {
            last_display_ms = now;
            speed = Encoder_GetSpeed();

            OLED_Clear();

            /* 第一行：目标速度 */
            OLED_ShowString(0, 0, "T:", 12);
            OLED_ShowNum(16, 0, SPEED_TARGET_CM_S, 3, 12);
            OLED_ShowString(40, 0, "cm/s", 12);

            /* 第二行：左轮速度 */
            OLED_ShowString(0, 16, "L:", 12);
            display_signed_tenths(16, 16, -speed.left * 100.0f);
            OLED_ShowString(80, 16, "cm/s", 12);

            /* 第三行：右轮速度（取反显示） */
            OLED_ShowString(0, 32, "R:", 12);
            display_signed_tenths(16, 32, speed.right * 100.0f);
            OLED_ShowString(80, 32, "cm/s", 12);

            /* 第四行：PWM 输出 */
            OLED_ShowString(0, 48, "PL:", 12);
            display_signed_int16(28, 48, pwm_left, 4);
            OLED_ShowString(68, 48, "PR:", 12);
            display_signed_int16(96, 48, pwm_right, 4);

            OLED_Refresh();
            DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
        }
    }
}

#endif /* TEST_SPEED_LOOP */

/* ================================================================
 *  TEST_HEADING_LOOP — 待实现
 * ================================================================ */
#ifdef TEST_HEADING_LOOP

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "HEADING LOOP", 12);
    OLED_ShowString(0, 16, "TODO", 12);
    OLED_Refresh();

    while (1) {
        __WFI();
    }
}

#endif /* TEST_HEADING_LOOP */

/* ================================================================
 *  TEST_LINE_TURN — 待实现
 * ================================================================ */
#ifdef TEST_LINE_TURN

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "LINE TURN", 12);
    OLED_ShowString(0, 16, "TODO", 12);
    OLED_Refresh();

    while (1) {
        __WFI();
    }
}

#endif /* TEST_LINE_TURN */
