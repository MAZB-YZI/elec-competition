/**
 * main.c — PID 巡线 + L 形直角 + 蓝牙调参
 *
 * 状态: NORMAL / TURN_L / TURN_R
 * 蓝牙: HC-05 → UART3 (PB2/TX, PB3/RX)
 */

#include "ti_msp_dl_config.h"
#include "gray_sensor.h"
#include "motor.h"
#include "oled.h"
#include "delay.h"
#include "bluetooth.h"
#include "jy61p.h"
#include "ir_sensor.h"
#include "buzzer.h"
#include "route_fsm.h"

/* ========== 按键 ========== */
#define BTN_COUNT       4
#define BTN_DEBOUNCE    3       /* 30ms 消抖 */
#define BTN_K1          0       /* PB1  - 上一题 */
#define BTN_K2          1       /* PB10 - 下一题 */
#define BTN_K3          2       /* PB11 - START */
#define BTN_K4          3       /* PB14 - STOP */

static const struct { GPIO_Regs *port; uint32_t pin; uint32_t iomux; } g_btn[BTN_COUNT] = {
    { GPIOB, DL_GPIO_PIN_1,  IOMUX_PINCM13 },
    { GPIOB, DL_GPIO_PIN_10, IOMUX_PINCM27 },
    { GPIOB, DL_GPIO_PIN_11, IOMUX_PINCM28 },
    { GPIOB, DL_GPIO_PIN_14, IOMUX_PINCM31 },
};
static uint8_t g_btn_deb[BTN_COUNT];
static bool    g_btn_pressed[BTN_COUNT];
static bool    g_btn_event[BTN_COUNT];

static void Buttons_Init(void)
{
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        /* 1. 关闭输出使能 */
        DL_GPIO_disableOutput(g_btn[i].port, g_btn[i].pin);
        /* 2. 配置引脚复用为 GPIO 输入上拉 */
        DL_GPIO_initDigitalInputFeatures(
            g_btn[i].iomux, DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP_DISABLE);
        /* 3. 清除输出使能位（确保为输入） */
        g_btn[i].port->DOECLR31_0 = g_btn[i].pin;
        g_btn_deb[i] = 0;
        g_btn_pressed[i] = false;
        g_btn_event[i] = false;
    }
}

static void Buttons_Scan(void)
{
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        bool raw_low = !DL_GPIO_readPins(g_btn[i].port, g_btn[i].pin);
        if (raw_low) {
            if (g_btn_deb[i] < BTN_DEBOUNCE) g_btn_deb[i]++;
            if (g_btn_deb[i] >= BTN_DEBOUNCE && !g_btn_pressed[i]) {
                g_btn_pressed[i] = true;
                g_btn_event[i] = true;
            }
        } else {
            g_btn_deb[i] = 0;
            g_btn_pressed[i] = false;
        }
    }
}

static bool Button_IsPressed(uint8_t btn)
{
    if (btn >= BTN_COUNT) return false;
    bool e = g_btn_event[btn];
    g_btn_event[btn] = false;
    return e;
}

/* ========== 常量 ========== */
#define LEFT_ENCODER_DIR   1      /* 左轮编码器方向系数 */
#define RIGHT_ENCODER_DIR -1     /* 右轮编码器方向系数，右轮安装反向 */

/* ISR ↔ main 共享 */
static volatile uint8_t  g_raw;          /* 灰度原始 8-bit */
static volatile int16_t  g_steer;        /* 当前转向修正量 */
static volatile float    g_yaw;          /* 当前航向角 */
static volatile uint32_t g_ms_ticks;
static volatile bool     g_gyro_ok;       /* JY61P 是否在线 */
static volatile uint32_t g_gyro_frames;  /* JY61P 帧计数 */

static int32_t    g_last_enc_l, g_last_enc_r; /* 编码器上次值 */
static volatile int16_t g_speed_l, g_speed_r; /* 编码器速度 脉冲/秒 */
static float      g_total_angle;              /* 全局累计角度 */
static float      g_last_yaw_for_total;       /* 上次yaw，用于全局累计 */
static uint32_t   last_oled_ms;               /* 上次 OLED 刷新时间 */
static uint32_t   last_btn_ms;               /* 上次按键扫描时间 */
static DL_SYSCTL_RESET_CAUSE g_reset_cause;   /* 启动后立即保存，避免复位原因丢失 */

/* 题目选择 */
static const RouteMode_t g_question_modes[] = {
    ROUTE_MODE_H_LAP,       /* Q2 */
    ROUTE_MODE_H_LAP,       /* Q3 (小车不动，通知相机) */
    ROUTE_MODE_Q4_AB,       /* Q4 */
    ROUTE_MODE_Q5_LAP,      /* Q5 */
};
#define QUESTION_COUNT 4
static uint8_t g_current_question = 0;  /* 0=Q2, 1=Q3, 2=Q4, 3=Q5 */

/* ================================================================
 *  辅助
 * ================================================================ */
static float yaw_diff(float current, float start)
{
    float diff = current - start;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

static int16_t clamp_pwm(int32_t value)
{
    if (value > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (value < MOTOR_PWM_MIN) return MOTOR_PWM_MIN;
    return (int16_t)value;
}

static void OLED_ShowSigned4(uint8_t x, uint8_t y, int32_t value)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        value = -value;
    } else {
        OLED_ShowString(x, y, "+", 12);
    }
    OLED_ShowNum((uint8_t)(x + 6), y, (uint32_t)value, 4, 12);
}

static void OLED_ShowBootStatus(const char *line)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "26Hcar JY61P", 16);
    OLED_ShowString(0, 20, line, 12);
    OLED_Refresh();
}

static const char *ResetCauseName(DL_SYSCTL_RESET_CAUSE cause)
{
    switch (cause) {
    case DL_SYSCTL_RESET_CAUSE_POR_HW_FAILURE:
    case DL_SYSCTL_RESET_CAUSE_BOR_SUPPLY_FAILURE:
        return "POWER";
    case DL_SYSCTL_RESET_CAUSE_POR_EXTERNAL_NRST:
    case DL_SYSCTL_RESET_CAUSE_BOOTRST_EXTERNAL_NRST:
        return "NRST";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_CPU_LOCKUP_VIOLATION:
        return "LOCKUP";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_DEBUG_TRIGGERED:
    case DL_SYSCTL_RESET_CAUSE_CPURST_DEBUG_TRIGGERED:
        return "DEBUG";
    case DL_SYSCTL_RESET_CAUSE_NO_RESET:
        return "NONE";
    default:
        return "OTHER";
    }
}

/* ================================================================
 *  TIMG6 ISR: 5ms 控制
 * ================================================================ */
void CTRL_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(CTRL_TIMER_INST)) {
    case DL_TIMER_IIDX_LOAD: {
        g_ms_ticks += 5U;
        /* 编码器测速（方向归一化：前进时都为正） */
        int32_t enc_l = Encoder_GetLeftCount();
        int32_t enc_r = Encoder_GetRightCount();
        g_speed_l = (int16_t)((enc_l - g_last_enc_l) * 200 * LEFT_ENCODER_DIR);
        g_speed_r = (int16_t)((enc_r - g_last_enc_r) * 200 * RIGHT_ENCODER_DIR);
        g_last_enc_l = enc_l; g_last_enc_r = enc_r;

        float   yaw   = JY61P_GetYaw();

        /* 全局累计角度：每5ms累加yaw变化，用于计算总圈数 */
        float total_delta = yaw_diff(yaw, g_last_yaw_for_total);
        g_last_yaw_for_total = yaw;
        g_total_angle += total_delta;

        uint8_t raw  = GraySensor_Read();

        /* ---- Route 状态机 (内部处理 START/STOP + 计时 + 所有模式) ---- */
        if (Route_Update5ms(g_ms_ticks, yaw, raw)) {
            g_steer = Route_GetLineSteer();
        }

        /* 每次 ISR 都更新显示变量 */
        g_raw = raw;
        g_yaw = yaw;
        break;
    }
    default:
        break;
    }
}

void GROUP1_IRQHandler(void) { Encoder_ISR(); }

/* UART0 中断 → JY61P 陀螺仪 */
void UART0_IRQHandler(void) { JY61P_UART_IRQHandler(); }

/* ================================================================
 *  main
 * ================================================================ */
int main(void)
{
    g_reset_cause = DL_SYSCTL_getResetCause();
    SYSCFG_DL_init();
    Motor_Init();
    Buzzer_Init();
    Route_Init();
    OLED_Init();
    OLED_ShowBootStatus("BOOT...");
    g_total_angle = 0.0f;
    g_gyro_ok = false;
    g_gyro_frames = 0U;
    last_oled_ms = 0;
    BT_Init();
    BT_SetTickPtr(&g_ms_ticks);
    Buttons_Init();

    OLED_ShowBootStatus("JY61P INIT...");
    JY61P_Init();
    delay_ms(500);          /* 等 JY61P 开始发送数据 */

    if (!JY61P_IsOnline()) {
        Motor_Stop();
        Buzzer_Stop();
        OLED_Clear();
        OLED_ShowString(0, 0, "JY61P FAIL", 12);
        OLED_ShowString(0, 16, "CHK TX/RX", 12);
        OLED_ShowString(0, 32, "PA0=TX PA1=RX", 12);
        OLED_Refresh();
        while (1) {
            Motor_Stop();
            Buzzer_Stop();
        }
    }

    OLED_ShowBootStatus("JY61P OK");
    delay_ms(200);
    JY61P_ZeroYaw();
    g_last_yaw_for_total = JY61P_GetYaw();
    g_yaw = g_last_yaw_for_total;
    g_gyro_ok = true;

    OLED_Clear();
    OLED_ShowString(0, 0, "Hcar JY61P", 16);
    OLED_Refresh();

    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(CTRL_TIMER_INST);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    BT_Printf("LinerCar Ready RST=%s(%u)\r\n",
              ResetCauseName(g_reset_cause), (unsigned)g_reset_cause);

    uint32_t last_bno_ms = g_ms_ticks;
    uint32_t last_tel_ms = g_ms_ticks;
    last_btn_ms = g_ms_ticks;

    while (1) {
        uint32_t now = g_ms_ticks;

        /* JY61P 通过 UART 中断自动更新，这里只刷新在线状态 */
        if ((now - last_bno_ms) >= 50U) {
            last_bno_ms = now;
            g_gyro_ok = JY61P_IsOnline();
            g_gyro_frames = JY61P_GetFrameCount();
        }

        /* 蓝牙命令处理 */
        BT_Poll();

        /* 按键扫描（每 10ms） */
        if ((now - last_btn_ms) >= 10U) {
            last_btn_ms = now;
            Buttons_Scan();

            /* K4 = STOP（任何状态有效） */
            if (Button_IsPressed(BTN_K4)) {
                g_route_stop_request = true;
            }

            /* K3 = START（仅待机状态有效） */
            if (Button_IsPressed(BTN_K3)) {
                if (!Route_IsActive()) {
                    Route_SetMode(g_question_modes[g_current_question]);
                    g_route_start_request = true;
                }
            }

            /* K1 = 上一题（仅待机状态） */
            if (Button_IsPressed(BTN_K1)) {
                if (!Route_IsActive()) {
                    if (g_current_question > 0) g_current_question--;
                    else g_current_question = QUESTION_COUNT - 1;
                }
            }

            /* K2 = 下一题（仅待机状态） */
            if (Button_IsPressed(BTN_K2)) {
                if (!Route_IsActive()) {
                    g_current_question = (g_current_question + 1) % QUESTION_COUNT;
                }
            }
        }

        /* 遥测输出 */
        if (BT_GetTelPeriod() > 0 && (now - last_tel_ms) >= BT_GetTelPeriod()) {
            last_tel_ms = now;
            BT_Telemetry(g_raw, g_speed_l, g_speed_r, g_yaw);
        }

        /* OLED: 固定 100ms 刷新 */
        if ((now - last_oled_ms) >= 100U) {
            last_oled_ms = now;
            uint8_t  raw = g_raw;
            RouteMode_t mode = Route_GetMode();

            OLED_ClearBuffer();

            /* 行0 y=0: 灰度8位二进制 + BC */
            for (uint8_t i = 0; i < 8; i++)
                OLED_ShowNum(i * 8, 0, (raw >> i) & 1, 1, 12);
            uint8_t bc = 0;
            for (uint8_t i = 0; i < 8; i++) bc += (raw >> i) & 1;
            OLED_ShowString(66, 0, "B", 12);
            OLED_ShowNum(74, 0, bc, 1, 12);

            /* 行1 y=13: 位置误差(P) + 转向(S) */
            OLED_ShowString(0, 13, "P", 12);
            OLED_ShowSigned4(8, 13, Route_GetLineError());
            OLED_ShowString(50, 13, "S", 12);
            OLED_ShowSigned4(58, 13, Route_GetLineSteer());

            /* 行2 y=26: 时间 + 题目 */
            OLED_ShowString(0, 26, "T", 12);
            uint32_t sec_x100 = Route_GetElapsedMs() / 10U;
            OLED_ShowNum(8, 26, sec_x100 / 100, 2, 12);
            OLED_ShowString(20, 26, ".", 12);
            OLED_ShowNum(26, 26, sec_x100 % 100, 2, 12);
            OLED_ShowString(44, 26, "s", 12);
            OLED_ShowString(56, 26, "Q", 12);
            OLED_ShowNum(64, 26, g_current_question + 2, 1, 12);

            /* 行3 y=39: 编码器速度 */
            OLED_ShowString(0, 39, "L", 12);
            OLED_ShowSigned4(8, 39, g_speed_l);
            OLED_ShowString(56, 39, "R", 12);
            OLED_ShowSigned4(64, 39, g_speed_r);

            /* 行4 y=52: 航向/结束原因 + 重启原因 */
            if (Route_IsFinished()) {
                OLED_ShowString(0, 52, Route_GetFinishReasonStr(), 12);
                OLED_ShowString(64, 52, ResetCauseName(g_reset_cause), 12);
            } else {
                OLED_ShowString(0, 52, "Y", 12);
                int32_t yaw_int = (int32_t)(g_yaw * 10);
                OLED_ShowSigned4(8, 52, yaw_int / 10);
                OLED_ShowString(64, 52, ResetCauseName(g_reset_cause), 12);
            }

            OLED_Refresh();
        }
    }
}
