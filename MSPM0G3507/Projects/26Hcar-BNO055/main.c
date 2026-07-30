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

/* ========== 默认参数 (蓝牙可改) ========== */
#define DEAD_ZONE   3             /* 位置死区: ±3 内不调 */
#define LOST_MS     2000          /* 丢线超时 ms */
#define STEER_SLEW_STEP 75        /* 5ms 内最大转向变化 */
#define CONTROL_DT 0.005f         /* 控制周期 5ms */
#define LEFT_ENCODER_DIR   1      /* 左轮编码器方向系数 */
#define RIGHT_ENCODER_DIR -1     /* 右轮编码器方向系数，右轮安装反向 */

static volatile float   g_KP         = 1.8f;   /* 位置比例 */
static volatile float   g_KI         = 0.0f;   /* 位置积分 */
static volatile float   g_KD         = 0.0f;   /* 位置微分 */
static volatile int16_t g_BASE_PWM   = 700;    /* 直行基准 PWM */
static volatile int16_t g_TURN_SPEED = 650;    /* 未使用，保留接口兼容 */
static volatile int16_t g_OUTPUT_LIM = 1000;   /* 转向输出限幅 */

/* ISR ↔ main 共享 */
static volatile uint8_t  g_raw;          /* 灰度原始 8-bit */
static volatile int16_t  g_steer;        /* 当前转向修正量 */
static volatile float    g_yaw;          /* 当前航向角 */
static volatile bool     g_new_data;     /* ISR 新数据标志 */
static volatile uint32_t g_ms_ticks;
static volatile bool     g_gyro_ok;       /* JY61P 是否在线 */
static volatile uint32_t g_gyro_frames;  /* JY61P 帧计数 */

static int16_t    g_last_steer;          /* 上次转向量(丢线保持用) */
static int16_t    g_last_pos_ctrl;       /* 上次位置误差，用于 D 项 */
static uint32_t   g_lost_cnt;            /* 丢线持续计数 */
static int32_t    g_last_enc_l, g_last_enc_r; /* 编码器上次值 */
static volatile int16_t g_speed_l, g_speed_r; /* 编码器速度 脉冲/秒 */
static float      g_total_angle;              /* 全局累计角度 */
static float      g_last_yaw_for_total;       /* 上次yaw，用于全局累计 */
static uint32_t   g_run_start_ms;             /* Route 启动时的 ms_ticks */
static uint32_t   g_run_elapsed_ms;           /* Route 运行耗时 */
static bool       g_stop_mode_active;         /* STOP 模式下是否已按 START */
static uint32_t   last_oled_ms;               /* 上次 OLED 刷新时间 */

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
    OLED_ShowString(0, 0, "24Hcar BNO", 16);
    OLED_ShowString(0, 20, line, 12);
    OLED_Refresh();
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

        /* ---- 处理 START/STOP 请求 (STOP 优先) ---- */
        if (g_route_stop_request) {
            g_route_stop_request = false;
            g_stop_mode_active = false;
            Motor_Stop();
            /* 冻结计时 */
            if (g_run_start_ms > 0U)
                g_run_elapsed_ms = g_ms_ticks - g_run_start_ms;
        }
        if (g_route_start_request) {
            g_route_start_request = false;
            g_run_start_ms = g_ms_ticks;
            g_run_elapsed_ms = 0;
            if (Route_GetMode() == ROUTE_MODE_STOP) {
                g_stop_mode_active = true;
            }
        }

        /* ---- Route 状态机 (route_fsm.c 内部读灰度+控电机) ---- */
        if (Route_Update5ms(yaw, raw)) {
            g_steer = Route_GetLineSteer();
            /* 更新运行计时 */
            if (Route_IsActive() && g_run_start_ms > 0U) {
                g_run_elapsed_ms = g_ms_ticks - g_run_start_ms;
            }
        }

        /* ---- Route 不活跃: STOP 模式下跑普通巡线 (需按 START) ---- */
        if (Route_GetMode() == ROUTE_MODE_STOP && g_stop_mode_active) {

        int8_t s[8]; uint8_t i;
        for (i = 0; i < 8; i++) s[i] = (raw >> i) & 1;
        int16_t pos = ( 200)*s[0] + (140)*s[1] + ( 75)*s[2] + ( 40)*s[3]
                    + (-40)*s[4] + (-75)*s[5] + (-140)*s[6] + (-200)*s[7];

        int16_t steer = 0;
        int16_t base  = g_BASE_PWM;
        int16_t lim   = g_OUTPUT_LIM;

        bool has_line = (s[0] || s[1] || s[2] || s[3] ||
                         s[4] || s[5] || s[6] || s[7]);
        bool all_black = (raw == 0xFF);

        if (has_line && !all_black) {
            /* 正常巡线 */
            int16_t pos_ctrl = pos;
            if (pos_ctrl > -DEAD_ZONE && pos_ctrl < DEAD_ZONE) {
                pos_ctrl = 0;
            }

            int16_t d_pos = pos_ctrl - g_last_pos_ctrl;
            steer = (int16_t)(-((float)pos_ctrl * g_KP + (float)d_pos * g_KD));
            g_last_pos_ctrl = pos_ctrl;

            if (steer >  lim) steer =  lim;
            if (steer < -lim) steer = -lim;

            int16_t delta = steer - g_last_steer;
            if (delta >  STEER_SLEW_STEP) steer = g_last_steer + STEER_SLEW_STEP;
            if (delta < -STEER_SLEW_STEP) steer = g_last_steer - STEER_SLEW_STEP;

            g_last_steer = steer;
            g_lost_cnt   = 0;
        } else {
            /* 丢线: 保持上次转向, 2s 停车 */
            g_last_pos_ctrl = 0;
            steer = g_last_steer;
            if (++g_lost_cnt > LOST_MS / 5) {
                Motor_Stop(); steer = 0;
            }
        }

        Motor_SetLeftSpeed(clamp_pwm((int32_t)base + steer));
        Motor_SetRightSpeed(clamp_pwm((int32_t)base - steer));
        g_steer = steer;

        } /* end ROUTE_MODE_STOP */

        /* 每次 ISR 都更新显示变量 */
        g_raw = raw;
        g_yaw = yaw;
        g_new_data = true;
        break;
    }
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
    SYSCFG_DL_init();
    Motor_Init();
    Buzzer_Init();
    Route_Init();
    OLED_Init();
    OLED_ShowBootStatus("BOOT...");
    g_total_angle = 0.0f;
    g_gyro_ok = false;
    g_gyro_frames = 0U;
    g_stop_mode_active = false;
    g_run_start_ms = 0;
    g_run_elapsed_ms = 0;
    last_oled_ms = 0;
    BT_Init();
    BT_SetTickPtr(&g_ms_ticks);

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
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    BT_Send("LinerCar Ready\r\n");

    TuningParams_t bt_params = { g_KP, g_KI, g_BASE_PWM, g_TURN_SPEED, g_OUTPUT_LIM };
    uint32_t last_bno_ms = g_ms_ticks;

    while (1) {
        uint32_t now = g_ms_ticks;

        /* JY61P 通过 UART 中断自动更新，这里只刷新在线状态 */
        if ((now - last_bno_ms) >= 50U) {
            last_bno_ms = now;
            g_gyro_ok = JY61P_IsOnline();
            g_gyro_frames = JY61P_GetFrameCount();
        }

        /* 蓝牙调参 */
        if (BT_Poll(&bt_params)) {
            g_KP         = bt_params.KP;
            g_KI         = bt_params.KI;
            g_BASE_PWM   = bt_params.BASE_PWM;
            g_TURN_SPEED = bt_params.TURN_SPEED;
            g_OUTPUT_LIM = bt_params.OUTPUT_LIM;
            /* 参数已直接使用，无需 PID_Init */
            /* 同步到 Route 状态机 */
            Route_SetBasePwm(bt_params.BASE_PWM);
            Route_SetKp(bt_params.KP);
            Route_SetKd(g_KD);
            Route_SetOutputLim(bt_params.OUTPUT_LIM);
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
            int16_t line_err, line_steer;
            if (Route_GetMode() != ROUTE_MODE_STOP) {
                line_err = Route_GetLineError();
                line_steer = Route_GetLineSteer();
            } else {
                line_err = g_steer;  /* STOP模式: steer本身就是误差映射 */
                line_steer = g_steer;
            }
            OLED_ShowString(0, 13, "P", 12);
            OLED_ShowSigned4(8, 13, line_err);
            OLED_ShowString(50, 13, "S", 12);
            OLED_ShowSigned4(58, 13, line_steer);

            /* 行2 y=26: 运行时间 + 模式 */
            OLED_ShowString(0, 26, "T", 12);
            uint32_t sec_x100 = g_run_elapsed_ms / 10U;
            OLED_ShowNum(8, 26, sec_x100 / 100, 2, 12);
            OLED_ShowString(20, 26, ".", 12);
            OLED_ShowNum(26, 26, sec_x100 % 100, 2, 12);
            OLED_ShowString(44, 26, "s", 12);
            OLED_ShowString(56, 26, "M", 12);
            OLED_ShowNum(64, 26, (uint32_t)mode, 1, 12);

            /* 行3 y=39: 编码器速度 */
            OLED_ShowString(0, 39, "L", 12);
            OLED_ShowSigned4(8, 39, g_speed_l);
            OLED_ShowString(56, 39, "R", 12);
            OLED_ShowSigned4(64, 39, g_speed_r);

            /* 行4 y=52: 航向/结束原因 */
            if (Route_IsFinished()) {
                OLED_ShowString(0, 52, Route_GetFinishReasonStr(), 12);
                OLED_ShowString(64, 52, g_gyro_ok ? "OK" : "ER", 12);
            } else {
                OLED_ShowString(0, 52, "Y", 12);
                int32_t yaw_int = (int32_t)(g_yaw * 10);
                OLED_ShowSigned4(8, 52, yaw_int / 10);
                OLED_ShowString(64, 52, g_gyro_ok ? "J61" : "ERR", 12);
            }

            OLED_Refresh();
        }
    }
}
