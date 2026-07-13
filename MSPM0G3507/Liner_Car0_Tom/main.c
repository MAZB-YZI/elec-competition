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

/* ========== 默认参数 (蓝牙可改) ========== */
#define KD          0.0f
#define DEAD_ZONE   20
#define LOST_MS     200
#define TURN_HOLD   200

static volatile float   g_KP         = 2.0f;
static volatile float   g_KI         = 0.1f;
static volatile int16_t g_BASE_PWM   = 1400;
static volatile int16_t g_TURN_SPEED = 2500;
static volatile int16_t g_OUTPUT_LIM = 1200;

/* 状态机 */
typedef enum { NORMAL, TURN_L, TURN_R } State_t;

/* ISR ↔ main 共享 */
static volatile uint8_t  g_raw;
static volatile int16_t  g_pos;
static volatile int16_t  g_steer;
static volatile State_t  g_state;
static volatile bool     g_new_data;

static PID_t      g_pid;
static int16_t    g_last_steer;
static uint32_t   g_lost_cnt;
static uint32_t   g_turn_ticks;

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
        uint8_t raw  = GraySensor_Read();
        int16_t pos  = GraySensor_GetPosition(raw);
        int16_t steer = 0;
        int16_t base  = g_BASE_PWM;
        int16_t turn  = g_TURN_SPEED;
        int16_t lim   = g_OUTPUT_LIM;

        State_t st = g_state;
        if (st == NORMAL) {
            State_t next = detect_turn(raw);
            if (next != NORMAL) { st = next; g_turn_ticks = 0; }
        } else {
            g_turn_ticks++;
            if (g_turn_ticks > TURN_HOLD / 5 && turn_done(raw))
                st = NORMAL;
        }
        g_state = st;

        if (st == TURN_L) {
            Motor_SetLeftSpeed ( turn);
            Motor_SetRightSpeed(-turn);
            steer = lim;
        } else if (st == TURN_R) {
            Motor_SetLeftSpeed (-turn);
            Motor_SetRightSpeed( turn);
            steer = -lim;
        } else {
            if (pos >= 0) {
                int16_t err = pos - 350;
                if (err > DEAD_ZONE || err < -DEAD_ZONE)
                    steer = PID_Compute(&g_pid, 0, err, 0.005f);
                g_last_steer = steer;
                g_lost_cnt   = 0;
            } else {
                steer = g_last_steer;
                if (++g_lost_cnt > LOST_MS / 5) {
                    Motor_Stop(); steer = 0;
                }
            }
            Motor_SetLeftSpeed (base - steer);
            Motor_SetRightSpeed(base + steer);
        }

        g_raw = raw; g_pos = pos; g_steer = steer;
        g_new_data = true;
        break;
    }
    }
}

void GROUP1_IRQHandler(void) { Encoder_ISR(); }

/* ================================================================
 *  main
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
    OLED_Init();
    PID_Init(&g_pid, g_KP, g_KI, KD, g_OUTPUT_LIM);
    BT_Init();
    BT_Send("LinerCar Ready\r\n");

    OLED_Clear();
    OLED_ShowString(0, 0, "LinerCar BT", 16);
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

                OLED_Clear();
                for (uint8_t i = 0; i < 8; i++)
                    OLED_ShowNum(i * 16, 0, (raw >> i) & 1, 1, 16);

                const char *ss = (st == NORMAL) ? "NORM"
                               : (st == TURN_L) ? "TL  " : "TR  ";
                OLED_ShowString(0, 18, ss, 12);
                if (pos >= 0) OLED_ShowNum(36, 18, pos, 3, 12);

                OLED_ShowString(0, 38, "L:", 12);
                OLED_ShowNum(18, 38, (uint32_t)(spd_L>0?spd_L:-spd_L), 4, 12);
                OLED_ShowString(70, 38, "R:", 12);
                OLED_ShowNum(88, 38, (uint32_t)(spd_R>0?spd_R:-spd_R), 4, 12);

                OLED_Refresh();
            }
        }
    }
}
