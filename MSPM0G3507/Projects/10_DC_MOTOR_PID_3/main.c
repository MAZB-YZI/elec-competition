/**
 * main.c — 定时器中断驱动 PID 巡线
 *
 * TIMG6 每 5ms 触发 ISR → 读灰度 → PID → 设电机
 * 主循环只刷 OLED
 */

#include "ti_msp_dl_config.h"
#include "gray_sensor.h"
#include "motor.h"
#include "oled.h"
#include "delay.h"

#define KP          2.0f
#define KI          0.1f
#define KD          0.0f
#define OUTPUT_LIM  1200
#define BASE_PWM    1400
#define DEAD_ZONE   20
#define LOST_MS     200

/* ISR 和主循环共享 */
static volatile uint8_t  g_raw;
static volatile int16_t  g_pos;
static volatile int16_t  g_steer;
static volatile bool     g_new_data;

static PID_t      g_pid;
static int16_t    g_last_steer;
static uint32_t   g_lost_cnt;

/* ================================================================
 *  TIMG6 ISR: 每 5ms
 * ================================================================ */
void CTRL_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(CTRL_TIMER_INST)) {
    case DL_TIMER_IIDX_LOAD: {
        uint8_t raw = GraySensor_Read();
        int16_t pos = GraySensor_GetPosition(raw);
        int16_t steer;

        if (pos >= 0) {
            int16_t err = pos - 350;
            if (err > DEAD_ZONE || err < -DEAD_ZONE)
                steer = PID_Compute(&g_pid, 0, err, 0.005f);
            else
                steer = 0;
            g_last_steer = steer;
            g_lost_cnt   = 0;
        } else {
            steer = g_last_steer;
            if (++g_lost_cnt > LOST_MS / 5) {
                Motor_Stop();
                steer = 0;
            }
        }

        Motor_SetLeftSpeed (BASE_PWM - steer);
        Motor_SetRightSpeed(BASE_PWM + steer);

        g_raw      = raw;
        g_pos      = pos;
        g_steer    = steer;
        g_new_data = true;
        break;
    }
    }
}

/* ================================================================
 *  编码器中断
 * ================================================================ */
void GROUP1_IRQHandler(void)
{
    Encoder_ISR();
}

/* ================================================================
 *  main
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
    OLED_Init();
    PID_Init(&g_pid, KP, KI, KD, OUTPUT_LIM);

    OLED_Clear();
    OLED_ShowString(0, 0, "LinerCar", 16);
    OLED_Refresh();

    /* 启用中断 (定时器已由 SysConfig 自启) */
    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);

    uint32_t tick = 0;

    while (1) {
        /* 等 ISR 通知新数据 */
        if (g_new_data) {
            g_new_data = false;
            if (++tick % 100 == 0) {   /* 500ms 刷一次 */
                uint8_t  raw   = g_raw;
                int16_t  pos   = g_pos;
                int16_t  steer = g_steer;
                int16_t  L     = BASE_PWM - steer;
                int16_t  R     = BASE_PWM + steer;
                int16_t  ds    = steer;

                OLED_Clear();

                uint8_t i;
                for (i = 0; i < 8; i++)
                    OLED_ShowNum(i * 16, 0, (raw >> i) & 1, 1, 16);

                if (pos >= 0) {
                    OLED_ShowString(0, 20, "P:", 12);
                    OLED_ShowNum(18, 20, (uint32_t)pos, 3, 12);
                } else {
                    OLED_ShowString(0, 20, "LOST", 12);
                }

                OLED_ShowString(0, 38, "S:", 12);
                if (ds < 0) { OLED_ShowString(18, 38, "-", 12); ds = -ds; }
                OLED_ShowNum(28, 38, (uint32_t)ds, 4, 12);

                OLED_ShowString(0, 56, "L:", 12);
                OLED_ShowNum(20, 56, (uint32_t)(L > 0 ? L : -L), 4, 12);
                OLED_ShowString(70, 56, "R:", 12);
                OLED_ShowNum(90, 56, (uint32_t)(R > 0 ? R : -R), 4, 12);

                OLED_Refresh();
            }
        }
    }
}
