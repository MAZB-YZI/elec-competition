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

/* ========== 默认参数 (蓝牙可改) ========== */
#define KD          0.0f          /* 微分系数(未使用) */
#define DEAD_ZONE   3             /* 位置死区: ±3 内不调 */
#define LOST_MS     200           /* 丢线超时 ms */
#define TURN_HOLD   420           /* 直角最大持续时间 ms */
#define TURN_MIN    170           /* 直角最短保持 ms */
#define TURN_CONFIRM 45           /* 中间探头连续确认时间 ms */
#define TURN_SPEED_H 1500         /* 直角转弯 PWM */
#define STEER_SLEW_STEP 80        /* 5ms 内最大转向变化，保证弯道跟得上 */

static volatile float   g_KP         = 3.0f;   /* 位置比例 */
static volatile float   g_KI         = 0.3f;   /* 位置积分 */
static volatile int16_t g_BASE_PWM   = 650;    /* 直行基准 PWM */
static volatile int16_t g_TURN_SPEED = 650;    /* 蓝牙可调转弯速度 */
static volatile int16_t g_OUTPUT_LIM = 1000;   /* 位置 PID 输出限幅 */

/* 航向中环 PID 参数 */
#define HEADING_KP      2.5f
#define HEADING_KI      0.0f
#define HEADING_KD      0.0f
#define HEADING_LIM     800     /* 航向 PID 输出限幅 */

/* 状态机 */
typedef enum { NORMAL, TURN_L, TURN_R } State_t;
static int16_t g_pos_filt;              /* filtered line position */

/* ISR ↔ main 共享 */
static volatile uint8_t  g_raw;          /* 灰度原始 8-bit */
static volatile int16_t  g_pos;          /* 黑线质心 0~700 */
static volatile int16_t  g_steer;        /* 当前转向修正量 */
static volatile float    g_yaw;          /* 当前航向角 */
static volatile State_t  g_state;        /* 状态机: NORMAL/TURN_L/TURN_R */
static volatile bool     g_new_data;     /* ISR 新数据标志 */

static PID_t      g_pid;                 /* 位置 PID (外环) */
static PID_t      g_heading_pid;         /* 航向 PID (中环) */
static float      g_target_yaw;          /* 目标航向角 (度) */
static int16_t    g_last_steer;          /* 上次转向量(丢线保持用) */
static uint32_t   g_lost_cnt;            /* 丢线持续计数 */
static uint32_t   g_turn_ticks;          /* 转弯持续 5ms 计数 */
static uint32_t   g_turn_confirm_ticks;  /* 直角退出确认计数 */
static int32_t    g_last_enc_l, g_last_enc_r; /* 编码器上次值 */
static volatile int16_t g_speed_l, g_speed_r; /* 编码器速度 pulses/sec */

/* ================================================================
 *  辅助
 * ================================================================ */
static inline uint8_t black(uint8_t raw, uint8_t i) { return (raw >> i) & 1; }

static State_t detect_turn(uint8_t raw)
{
    if (black(raw,0) && !black(raw,4) && !black(raw,5)
                     && !black(raw,6) && !black(raw,7))
        return TURN_L;
    if (black(raw,7) && !black(raw,0) && !black(raw,1)
                     && !black(raw,2) && !black(raw,3))
        return TURN_R;
    return NORMAL;
}

static bool turn_done(uint8_t raw)
{
    uint8_t mid = black(raw,2) + black(raw,3)
                + black(raw,4) + black(raw,5);
    return (mid >= 2);
}
/* ================================================================
 *  TIMG6 ISR: 5ms 控制
 * ================================================================ */
void CTRL_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(CTRL_TIMER_INST)) {
    case DL_TIMER_IIDX_LOAD: {
        JY61P_UpdateTick();

        /* 编码器测速 */
        int32_t enc_l = Encoder_GetLeftCount();
        int32_t enc_r = Encoder_GetRightCount();
        g_speed_l = (int16_t)((enc_l - g_last_enc_l) * 200);
        g_speed_r = (int16_t)((enc_r - g_last_enc_r) * 200);
        g_last_enc_l = enc_l; g_last_enc_r = enc_r;

        uint8_t raw  = GraySensor_Read();

        /* 加权连续位置: 外层权重高 → 更细腻, 不会蹦 100 一跳 */
        int8_t s[8]; uint8_t i;
        for (i = 0; i < 8; i++) s[i] = (raw >> i) & 1;
        int16_t pos = ( 160)*s[0] + (100)*s[1] + ( 55)*s[2] + ( 30)*s[3]
                    + (-30)*s[4] + (-55)*s[5] + (-100)*s[6] + (-160)*s[7];

        int16_t steer = 0;
        int16_t base  = g_BASE_PWM;
        int16_t turn  = g_TURN_SPEED;
        int16_t lim   = g_OUTPUT_LIM;
        float   yaw   = JY61P_GetYaw();

        /* ---- 状态机: 直角检测 ---- */
        State_t st = g_state;
        if (st == NORMAL) {
            State_t next = detect_turn(raw);
            if (next != NORMAL) {
                st = next;
                g_turn_ticks = 0;
                g_turn_confirm_ticks = 0;
                /* 进入直角: 锁定当前航向, 不跟踪灰度 */
            }
        } else {
            g_turn_ticks++;
            if (turn_done(raw)) {
                g_turn_confirm_ticks++;
            } else {
                g_turn_confirm_ticks = 0;
            }
            if ((g_turn_ticks > (TURN_MIN/5) && g_turn_confirm_ticks > (TURN_CONFIRM/5))
                || g_turn_ticks > (TURN_HOLD/5)) {
                st = NORMAL;
                /* 退出直角: 重置目标航向为当前实际航向, 防突变 */
                g_target_yaw = yaw;
                PID_Reset(&g_heading_pid);
            }
        }
        g_state = st;

        /* ---- 执行 ---- */
        if (st == TURN_L) {
            Motor_SetLeftSpeed (-turn);
            Motor_SetRightSpeed(turn);
            steer = lim;
        } else if (st == TURN_R) {
            Motor_SetLeftSpeed (turn);
            Motor_SetRightSpeed(-turn);
            steer = -lim;
        } else {
            /* NORMAL: 灰度 P 巡线，无滤波避免滞后超调 */
            if (s[0] || s[1] || s[2] || s[3] || s[4] || s[5] || s[6] || s[7]) {
                int16_t pos_ctrl = pos;
                if (pos_ctrl > -DEAD_ZONE && pos_ctrl < DEAD_ZONE) {
                    pos_ctrl = 0;
                }

                steer = (int16_t)(-(float)pos_ctrl * g_KP);
                g_pos_filt = pos_ctrl;
                if (steer >  lim) steer =  lim;
                if (steer < -lim) steer = -lim;

                int16_t delta = steer - g_last_steer;
                if (delta >  STEER_SLEW_STEP) steer = g_last_steer + STEER_SLEW_STEP;
                if (delta < -STEER_SLEW_STEP) steer = g_last_steer - STEER_SLEW_STEP;

                g_target_yaw = yaw;
                g_last_steer = steer;
                g_lost_cnt   = 0;
            } else {
                /* 丢线: 保持上次转向, 超时停车 */
                steer = g_last_steer;
                if (++g_lost_cnt > LOST_MS / 5) {
                    Motor_Stop(); steer = 0;
                }
            }
            Motor_SetLeftSpeed (base + steer);
            Motor_SetRightSpeed(base - steer);
        }

        g_raw = raw; g_pos = pos; g_steer = steer; g_yaw = yaw;
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
    OLED_Init();
    PID_Init(&g_pid, g_KP, g_KI, KD, g_OUTPUT_LIM);
    PID_Init(&g_heading_pid, HEADING_KP, HEADING_KI, HEADING_KD, HEADING_LIM);
    g_target_yaw = 0.0f;
    BT_Init();
    JY61P_Init();
    BT_Send("LinerCar Ready\r\n");

    OLED_Clear();
    OLED_ShowString(0, 0, "LinerCar BT", 16);
    OLED_Clear();
    OLED_Refresh();

    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);

    TuningParams_t bt_params = { g_KP, g_KI, g_BASE_PWM, g_TURN_SPEED, g_OUTPUT_LIM };
    uint32_t tick = 0;

    while (1) {
        /* 蓝牙调参 */
        if (BT_Poll(&bt_params)) {
            g_KP         = bt_params.KP;
            g_KI         = bt_params.KI;
            g_BASE_PWM   = bt_params.BASE_PWM;
            g_TURN_SPEED = bt_params.TURN_SPEED;
            g_OUTPUT_LIM = bt_params.OUTPUT_LIM;
            PID_Init(&g_pid, g_KP, g_KI, KD, g_OUTPUT_LIM);
        }

        /* OLED */
        if (g_new_data) {
            g_new_data = false;
            if (++tick % 100 == 0) {
                uint8_t  raw = g_raw;
                int16_t  pos = g_pos;
                State_t  st  = g_state;
                int16_t  spd_L, spd_R;
                if (st == TURN_L)      { spd_L =  g_TURN_SPEED; spd_R = -g_TURN_SPEED; }
                else if (st == TURN_R) { spd_L = -g_TURN_SPEED; spd_R =  g_TURN_SPEED; }
                else {
                    int16_t s = g_steer;
                    spd_L = g_BASE_PWM - s;
                    spd_R = g_BASE_PWM + s;
                }

                for (uint8_t i = 0; i < 8; i++)
                    OLED_ShowNum(i * 16, 0, (raw >> i) & 1, 1, 16);
                
                const char *ss = (st == NORMAL) ? "NORM"
                               : (st == TURN_L) ? "TL  " : "TR  ";
                OLED_ShowString(0, 18, ss, 12);
                if (pos >= 0) OLED_ShowNum(36, 18, pos, 3, 12);

                /* IR 测距值 */
                OLED_ShowString(56, 18, "IR:", 12);
                OLED_ShowNum(74, 18, IR_Read(), 4, 12);

                OLED_ShowString(0, 38, "SL:", 12);
                OLED_ShowNum(20, 38, (uint32_t)(g_speed_l>0?g_speed_l:-g_speed_l), 4, 12);
                OLED_ShowString(60, 38, "SR:", 12);
                OLED_ShowNum(80, 38, (uint32_t)(g_speed_r>0?g_speed_r:-g_speed_r), 4, 12);
                
                /* Yaw: 实际航向 / 目标航向 */
                OLED_ShowString(0, 52, "Y:", 12);
                float yaw = g_yaw;
                int32_t yaw_int = (int32_t)(yaw * 10);
                if (yaw_int < 0) {
                    OLED_ShowString(16, 52, "-", 12);
                    OLED_ShowNum(22, 52, (uint32_t)(-yaw_int) / 10, 3, 12);
                } else {
                    OLED_ShowString(16, 52, "+", 12);
                    OLED_ShowNum(22, 52, (uint32_t)yaw_int / 10, 3, 12);
                }
                OLED_ShowString(40, 52, ".", 12);
                OLED_ShowNum(46, 52, (uint32_t)(yaw_int > 0 ? yaw_int : -yaw_int) % 10, 1, 12);
                OLED_ShowString(56, 52, "/", 12);
                int32_t tgt_int = (int32_t)(g_target_yaw * 10);
                if (tgt_int < 0) {
                    OLED_ShowString(64, 52, "-", 12);
                    OLED_ShowNum(70, 52, (uint32_t)(-tgt_int) / 10, 3, 12);
                } else {
                    OLED_ShowString(64, 52, "+", 12);
                    OLED_ShowNum(70, 52, (uint32_t)tgt_int / 10, 3, 12);
                }

                OLED_Refresh();
            }
        }
    }
}
