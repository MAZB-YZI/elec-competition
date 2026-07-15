/**
 * main.c — 功能测试
 * 生产代码见末尾注释
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "oled.h"
#include "gray_sensor.h"
#include "delay.h"

int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
    OLED_Init();

    OLED_Clear();
    OLED_ShowString(0, 0, "OLED: Ready", 16);
    OLED_ShowString(0, 16, "Motor: FWD", 16);
    OLED_ShowString(0, 32, "Gray:", 16);
    OLED_Refresh();

   
    // Motor_SetRightSpeed(1000);   /* PWMA=A12 右电机 */
    // Motor_SetLeftSpeed(1000);    /* PWMB=A13 左电机 */

    while (1) {
        uint8_t raw = GraySensor_Read();
        char buf[20]; uint8_t i;
        OLED_Clear();
        OLED_ShowString(0, 0, "OLED: Ready", 16);
        OLED_ShowString(0, 16, "Motor: FWD", 16);
        for (i = 0; i < 8; i++) buf[i] = (raw & (1 << i)) ? '1' : '0';
        buf[8] = '\0';
        OLED_ShowString(0, 32, "Gray:", 16);
        OLED_ShowString(0, 48, buf, 16);
        OLED_Refresh();
        delay_ms(200);
    }
}

/* ================================================================
 *  生产代码 (PRODUCTION)
 * ================================================================
 **/
#include "motor.h"
#include "oled.h"
#include "gray_sensor.h"

#define SYS_FREQ_HZ   32000000U
#define CTRL_DT       0.01f
#define PID_KP        2.0f
#define PID_KI        0.5f
#define PID_KD        0.0f
#define PID_LIMIT     3000
#define LINE_CENTER   350
#define STEER_KP      3.0f
#define BASE_SPEED    200

static PID_t g_pid_left, g_pid_right;

int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
    OLED_Init();
    PID_Init(&g_pid_left,  PID_KP, PID_KI, PID_KD, PID_LIMIT);
    PID_Init(&g_pid_right, PID_KP, PID_KI, PID_KD, PID_LIMIT);

    OLED_Clear();
    OLED_ShowString(0, 0, "Liner Car V1.0", 16);
    OLED_Refresh();

    while (1) {
        uint8_t gray_raw = GraySensor_Read();
        int16_t position = GraySensor_GetPosition();

        static int32_t last_enc_l = 0, last_enc_r = 0;
        int32_t enc_l = Encoder_GetLeftCount();
        int32_t enc_r = Encoder_GetRightCount();
        int16_t speed_l = (int16_t)((enc_l - last_enc_l) * 100);
        int16_t speed_r = (int16_t)((enc_r - last_enc_r) * 100);
        last_enc_l = enc_l; last_enc_r = enc_r;

        int16_t steer = 0;
        if (position >= 0)
            steer = (int16_t)((float)(position - LINE_CENTER) * STEER_KP);
        int16_t target_l = BASE_SPEED - steer;
        int16_t target_r = BASE_SPEED + steer;

        int16_t pid_l = PID_Compute(&g_pid_left,  target_l, speed_l, CTRL_DT);
        int16_t pid_r = PID_Compute(&g_pid_right, target_r, speed_r, CTRL_DT);
        Motor_SetLeftSpeed(pid_l);
        Motor_SetRightSpeed(pid_r);

        static uint32_t frame = 0;
        if (++frame % 50 == 0) {
            char buf[22]; uint8_t i;
            for (i = 0; i < 8; i++)
                buf[i] = (gray_raw & (1 << i)) ? '_' : '#';
            buf[8] = '\0';
            OLED_Clear();
            OLED_ShowString(0, 0, "Liner Car V1.0", 16);
            OLED_ShowString(0, 16, buf, 12);
            if (position >= 0) OLED_ShowNum(0, 32, position, 3, 12);
            OLED_ShowString(0, 48, "PID RUNNING", 12);
            OLED_Refresh();
        }
        delay_ms(10);
    }
}


