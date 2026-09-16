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
#include "key.h"

/* ========== 常量 ========== */
#define LEFT_ENCODER_DIR   1      /* 左轮编码器方向系数 */
#define RIGHT_ENCODER_DIR -1     /* 右轮编码器方向系数，右轮安装反向 */

/* ========== Q5 单字节加速度前馈链路 ==========
 * UART_CONSOLE TX: PA23, 115200, 8N1 -> MaixCAM2 UART4 RX(A22).
 * bit7: 1=直道, 0=弯道; bit6..0: 纵向加速度线性映射到 0..127.
 *
 * 当前按实物判断采用“车头方向 = IMU -Y”。如果实车前推测试的符号
 * 相反，只需把 FF_FORWARD_AXIS_SIGN 改为 +1.0f。
 */
#define FF_TX_PERIOD_MS             20U
#define FF_ACCEL_RANGE_MPS2         4.0f
#define FF_ACCEL_FILTER_ALPHA       0.35f
#define FF_ACCEL_BIAS_ALPHA         0.02f
#define FF_FORWARD_AXIS_SIGN       (-1.0f)

#define Q5_STRAIGHT_CM              150.0f
#define Q5_CURVE_CM                 157.08f
#define Q5_ROUTE_GUARD_CM             5.0f

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
static DL_SYSCTL_RESET_CAUSE g_reset_cause;   /* 启动后立即保存，避免复位原因丢失 */

/* Q5 前馈发送状态。待机时估计静态零偏，运行时冻结。 */
static float      g_ff_accel_bias_mps2;
static volatile float g_ff_accel_filtered_mps2;
static bool       g_ff_bias_valid;
static volatile uint8_t    g_ff_last_byte;
static volatile bool       g_ff_last_straight;
static volatile uint32_t   g_ff_tx_count;
static volatile uint32_t   g_ff_drop_count;
static uint8_t    g_ff_div;               /* 5ms 分频计数 */

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

static float clamp_f(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float FF_ReadForwardAccelMps2(void)
{
    return FF_FORWARD_AXIS_SIGN * JY61P_GetAccelYMps2();
}

static bool FF_Q5IsStraight(void)
{
    if (Route_GetMode() != ROUTE_MODE_Q5_LAP || !Route_IsActive()) {
        return true;
    }

    const float d = Encoder_GetAverageDistanceCm();
    const float straight1_end = Q5_STRAIGHT_CM - Q5_ROUTE_GUARD_CM;
    const float straight2_start = Q5_STRAIGHT_CM + Q5_CURVE_CM +
                                  Q5_ROUTE_GUARD_CM;
    const float straight2_end = 2.0f * Q5_STRAIGHT_CM + Q5_CURVE_CM -
                                Q5_ROUTE_GUARD_CM;

    return (d < straight1_end) ||
           (d >= straight2_start && d < straight2_end);
}

static uint8_t FF_EncodeByte(float accel_mps2, bool is_straight)
{
    float a = clamp_f(accel_mps2,
                      -FF_ACCEL_RANGE_MPS2, FF_ACCEL_RANGE_MPS2);
    int32_t q;

    /* code=64 精确表示0；负半轴64级，正半轴63级。 */
    if (a <= 0.0f) {
        q = (int32_t)(64.0f +
            a * (64.0f / FF_ACCEL_RANGE_MPS2) + 0.5f);
    } else {
        q = 64 + (int32_t)(a * (63.0f / FF_ACCEL_RANGE_MPS2) + 0.5f);
    }

    if (q < 0) q = 0;
    if (q > 127) q = 127;
    return (uint8_t)((is_straight ? 0x80U : 0x00U) |
                     ((uint8_t)q & 0x7FU));
}

static void FF_SendByte(uint8_t byte)
{
    /* 非阻塞单次尝试，ISR 中不能循环等待 */
    if (DL_UART_Main_transmitDataCheck(UART_CONSOLE_INST, byte)) {
        g_ff_tx_count++;
        g_ff_last_byte = byte;
    } else {
        g_ff_drop_count++;
    }
}

static void FF_UpdateAndSend(void)
{
    float sample = 0.0f;

    /* 新鲜度检查: 超过 200ms 没收到 0x51 → 立即归零 */
    bool accel_fresh = JY61P_IsAccelFresh();

    if (!accel_fresh) {
        /* 失效: 立即归零，不经过滤波衰减 */
        g_ff_accel_filtered_mps2 = 0.0f;
    } else if (JY61P_HasAcceleration()) {
        float sample = FF_ReadForwardAccelMps2();

        if (!Route_IsActive()) {
            if (!g_ff_bias_valid) {
                g_ff_accel_bias_mps2 = sample;
                g_ff_bias_valid = true;
            } else {
                g_ff_accel_bias_mps2 += FF_ACCEL_BIAS_ALPHA *
                    (sample - g_ff_accel_bias_mps2);
            }
        }

        if (g_ff_bias_valid) sample -= g_ff_accel_bias_mps2;

        sample = clamp_f(sample,
                         -FF_ACCEL_RANGE_MPS2, FF_ACCEL_RANGE_MPS2);
        g_ff_accel_filtered_mps2 += FF_ACCEL_FILTER_ALPHA *
            (sample - g_ff_accel_filtered_mps2);
    }

    g_ff_last_straight = FF_Q5IsStraight();
    FF_SendByte(FF_EncodeByte(g_ff_accel_filtered_mps2,
                              g_ff_last_straight));
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
        JY61P_UpdateTick();
        Key_Scan5ms();

        /* 前馈发送: 4 分频 = 20ms / 50Hz */
        if (++g_ff_div >= 4U) {
            g_ff_div = 0;
            FF_UpdateAndSend();
        }
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
    g_ff_accel_bias_mps2 = 0.0f;
    g_ff_accel_filtered_mps2 = 0.0f;
    g_ff_bias_valid = false;
    g_ff_last_byte = 0xC0U;
    g_ff_last_straight = true;
    g_ff_tx_count = 0U;
    g_ff_drop_count = 0U;
    g_ff_div = 0U;
    last_oled_ms = 0;
    BT_Init();
    BT_SetTickPtr(&g_ms_ticks);
    Key_Init();

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
    BT_Printf("LinerCar Ready RST=%s(%u) FF=PA23/115200 AXIS=-Y "
              "RANGE=+/-%.1fmps2 A51=%lu\r\n",
              ResetCauseName(g_reset_cause), (unsigned)g_reset_cause,
              (double)FF_ACCEL_RANGE_MPS2,
              (unsigned long)JY61P_GetAccelFrameCount());

    uint32_t last_bno_ms = g_ms_ticks;
    uint32_t last_tel_ms = g_ms_ticks;

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

        /* 按键处理 */
        /* K4 = STOP（任何状态有效） */
        if (Key_GetPressEvent(KEY_ID_K4)) {
            g_route_stop_request = true;
        }

        /* K3 = START（仅待机状态有效） */
        if (Key_GetPressEvent(KEY_ID_K3)) {
            if (!Route_IsActive()) {
                Route_SetMode(g_question_modes[g_current_question]);
                g_route_start_request = true;
            }
        }

        /* K1 = 上一题（仅待机状态） */
        if (Key_GetPressEvent(KEY_ID_K1)) {
            if (!Route_IsActive()) {
                if (g_current_question > 0) g_current_question--;
                else g_current_question = QUESTION_COUNT - 1;
            }
        }

        /* K2 = 下一题（仅待机状态） */
        if (Key_GetPressEvent(KEY_ID_K2)) {
            if (!Route_IsActive()) {
                g_current_question = (g_current_question + 1) % QUESTION_COUNT;
            }
        }

        /* 遥测输出 */
        if (BT_GetTelPeriod() > 0 && (now - last_tel_ms) >= BT_GetTelPeriod()) {
            last_tel_ms = now;
            BT_Telemetry(g_raw, g_speed_l, g_speed_r, g_yaw);
            BT_Printf("FF AFWD=%.3f FILT=%.3f ROUTE=%c BYTE=0x%02X "
                      "A51=%lu TX=%lu DROP=%lu FRESH=%u\r\n",
                      (double)FF_ReadForwardAccelMps2(),
                      (double)g_ff_accel_filtered_mps2,
                      g_ff_last_straight ? 'S' : 'C',
                      (unsigned)g_ff_last_byte,
                      (unsigned long)JY61P_GetAccelFrameCount(),
                      (unsigned long)g_ff_tx_count,
                      (unsigned long)g_ff_drop_count,
                      (unsigned)JY61P_IsAccelFresh());
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
