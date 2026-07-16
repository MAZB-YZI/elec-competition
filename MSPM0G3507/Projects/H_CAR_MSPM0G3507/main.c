/**
 * main.c — H_CAR 测试主入口
 *
 * 通过宏切换不同测试模式：
 *   TEST_SPEED_LOOP    定速直行（速度闭环）
 *   TEST_DIST_MODE     定距直行停车（A→B 模拟）
 *   TEST_HEADING_LOOP  定航向直行（yaw 闭环）— 待实现
 *   TEST_LINE_TURN     直角循迹转弯 — 待实现
 */

#include "ti_msp_dl_config.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "oled.h"
#include "buzzer.h"
#include "jy61p.h"
#include "control.h"
#include "delay.h"

/* ========== 测试模式选择（取消注释想要的模式） ========== */
// #define TEST_SPEED_LOOP   /* 定速直行 */
#define TEST_DIST_MODE      /* 定距1米停车 */
// #define TEST_HEADING_LOOP /* 航向保持 */
// #define TEST_LINE_TURN    /* 循迹转弯 */

/* ========== 路线参数（方便调参） ========== */
#define LOOP_MS             10      /* 控制周期 ms */

/* 直行段参数 */
#define STRAIGHT_SPEED_MPS  0.40f   /* 直行速度 m/s */
#define STRAIGHT_DIST_M     1.01f   /* A→B 距离 m（编码器校准：实际1.0m） */
#define DIAGONAL_DIST_M     1.28f   /* A→C 对角线距离 m */

/* 速度环 PI（Motion 控制用） */
#define SPEED_KP            1500.0f
#define SPEED_KI            300.0f

/* 航向环 PD（直行保持用） */
#define HEADING_KP          1.0f
#define HEADING_KD          0.1f
#define HEADING_LIM         200.0f

/* 循迹环 P（弧线循迹用） */
#define LINE_KP             300.0f
#define LINE_LIM            500.0f
#define LINE_SPEED_MPS      0.16f   /* 循迹速度 m/s */

/* 弧线参数 */
#define ARC_LENGTH_M        1.26f   /* 半圆弧长 π×0.4m */

/* 蜂鸣器 */
#define BEEP_MS             1000U   /* 点位提示时长 ms */

/* ========== 速度环测试参数 ========== */
#define SPEED_TARGET_CM_S   50      /* 目标速度 cm/s */
#define SPEED_KP_TEST       8.0f    /* 速度环测试用 KP */
#define SPEED_KI_TEST       1.0f    /* 速度环测试用 KI */
#define SPEED_KD_TEST       0.0f
#define SPEED_PWM_LIMIT     2500

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
        Buzzer_Update1ms();
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

    PID_Init(&pid_left, SPEED_KP_TEST, SPEED_KI_TEST, SPEED_KD_TEST, SPEED_PWM_LIMIT);
    PID_Init(&pid_right, SPEED_KP_TEST, SPEED_KI_TEST, SPEED_KD_TEST, SPEED_PWM_LIMIT);

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

            Encoder_Update(dt);
            speed = Encoder_GetSpeed();

            float target_m_s = (float)SPEED_TARGET_CM_S / 100.0f;

            pwm_left = PID_Compute(&pid_left,
                (int16_t)(target_m_s * 1000),
                (int16_t)(speed.left * 1000),
                dt);
            pwm_right = PID_Compute(&pid_right,
                (int16_t)(target_m_s * 1000),
                (int16_t)(speed.right * 1000),
                dt);

            Motor_SetLeftSpeed(pwm_left);
            Motor_SetRightSpeed(pwm_right);

            ++loop_count;
        }

        /* 200ms 刷新显示 */
        if ((now - last_display_ms) >= 200U) {
            last_display_ms = now;
            speed = Encoder_GetSpeed();
            float speed_diff = speed.left - speed.right;

            OLED_Clear();
            OLED_ShowString(0, 0, "T:", 12);
            OLED_ShowNum(16, 0, SPEED_TARGET_CM_S, 3, 12);
            OLED_ShowString(40, 0, "cm/s", 12);
            OLED_ShowString(64, 0, "N:", 12);
            OLED_ShowNum(80, 0, loop_count, 6, 12);

            OLED_ShowString(0, 16, "L:", 12);
            display_signed_tenths(16, 16, -speed.left * 100.0f);
            OLED_ShowString(64, 16, "R:", 12);
            display_signed_tenths(80, 16, -speed.right * 100.0f);

            OLED_ShowString(0, 32, "D:", 12);
            display_signed_tenths(16, 32, -speed_diff * 100.0f);
            OLED_ShowString(80, 32, "cm/s", 12);

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
 *  TEST_DIST_MODE — 定距直行停车（A→B 模拟）
 * ================================================================ */
#ifdef TEST_DIST_MODE

int main(void)
{
    EncoderPair speed;
    EncoderPair dist;
    uint32_t last_loop_ms = 0U;
    uint32_t last_display_ms = 0U;
    uint32_t loop_count = 0U;
    bool arrived = false;
    uint32_t arrive_time = 0U;

    /* 初始化 */
    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    OLED_Init();
    Buzzer_Init();

    /* 启动编码器中断 */
    DL_GPIO_clearInterruptStatus(GPIOA,
        ENCODER1_A_ENC1_A_PIN | ENCODER2_A_ENC2_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER1_A_ENC1_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER2_A_ENC2_A_PIN);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /* 启动 1ms 定时器 */
    NVIC_ClearPendingIRQ(SYS_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(SYS_TICK_INST_INT_IRQN);

    /* JY61P 陀螺仪初始化：等收到足够帧再启动 */
    OLED_Clear();
    OLED_ShowString(0, 0, "DIST TEST", 12);
    OLED_ShowString(0, 16, "WAIT JY61P...", 12);
    OLED_Refresh();

    JY61P_Init();

    /* 等 JY61P 收到至少 50 帧（约 500ms 稳定数据） */
    while (JY61P_GetFrameCount() < 50U) {
        OLED_ShowNum(0, 32, JY61P_GetFrameCount(), 4, 12);
        OLED_Refresh();
        delay_ms(50U);
    }

    JY61P_ResetYaw();

    /* 设置控制参数 */
    Motion_SetSpeedGains(SPEED_KP, SPEED_KI);
    Motion_SetHeadingGains(HEADING_KP, HEADING_KD, HEADING_LIM);
    Motion_SetLineGains(LINE_KP, LINE_LIM);

    /* 启动运动 */
    OLED_Clear();
    OLED_ShowString(0, 0, "DIST TEST", 12);
    OLED_ShowString(0, 16, "DRIVING...", 12);
    OLED_Refresh();

    Motion_Init();
    Motion_DriveDistance(STRAIGHT_DIST_M, STRAIGHT_SPEED_MPS);

    last_loop_ms = g_ms_ticks;
    last_display_ms = g_ms_ticks;

    while (1) {
        uint32_t now = g_ms_ticks;

        /* 10ms 控制周期 */
        if ((now - last_loop_ms) >= LOOP_MS) {
            float dt = (float)(now - last_loop_ms) / 1000.0f;
            last_loop_ms = now;

            Encoder_Update(dt);
            Motion_Update10ms();

            ++loop_count;

            /* 检测到达 */
            if (!arrived && Motion_IsComplete()) {
                arrived = true;
                arrive_time = now;
                Motor_Stop();
                Buzzer_Beep(BEEP_MS);
            }
        }

        /* 200ms 刷新显示 */
        if ((now - last_display_ms) >= 200U) {
            last_display_ms = now;
            speed = Encoder_GetSpeed();
            dist = Encoder_GetDistance();
            float avg_dist = (fabsf(dist.left) + fabsf(dist.right)) * 0.5f;

            OLED_Clear();

            if (arrived) {
                OLED_ShowString(0, 0, "ARRIVED!", 12);
                OLED_ShowString(0, 16, "TIME:", 12);
                OLED_ShowNum(40, 16, (arrive_time) / 1000U, 2, 12);
                OLED_ShowString(56, 16, ".", 12);
                OLED_ShowNum(64, 16, (arrive_time % 1000U) / 100U, 1, 12);
                OLED_ShowString(72, 16, "s", 12);
            } else {
                OLED_ShowString(0, 0, "DRIVING...", 12);
            }

            /* 第二行：距离 */
            OLED_ShowString(0, 16, "D:", 12);
            display_signed_tenths(16, 16, avg_dist * 100.0f);
            OLED_ShowString(64, 16, "/", 12);
            display_signed_tenths(72, 16, STRAIGHT_DIST_M * 100.0f);
            OLED_ShowString(112, 16, "cm", 12);

            /* 第三行：左右速度 */
            OLED_ShowString(0, 32, "L:", 12);
            display_signed_tenths(16, 32, -speed.left * 100.0f);
            OLED_ShowString(64, 32, "R:", 12);
            display_signed_tenths(80, 32, -speed.right * 100.0f);

            /* 第四行：yaw */
            OLED_ShowString(0, 48, "YAW:", 12);
            display_signed_tenths(32, 48, JY61P_GetYaw());

            OLED_Refresh();
            DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
        }
    }
}

#endif /* TEST_DIST_MODE */

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
