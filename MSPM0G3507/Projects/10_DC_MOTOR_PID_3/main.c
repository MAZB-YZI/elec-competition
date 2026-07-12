/**
 * 双电机测试入口
 * PB21 切换模式
 *
 * 模式 0: 全停
 * 模式 1: 双电机正转
 * 模式 2: 双电机反转
 * 模式 3: 左正右反
 */

#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "drivers/key.h"
#include "drivers/motor.h"

extern volatile int32_t counter_1_A;
extern volatile int32_t counter_2_A;
static int32_t g_test_left_pwm = 0;
static int32_t g_test_right_pwm = 0;
static int32_t g_target_speed_left = 0;
static int32_t g_target_speed_right = 0;
extern int32_t PWM_1_duty;
extern int32_t PWM_2_duty;
extern float speed_1;
extern float speed_2;

static void reset_motion_state(void)
{
    DL_Timer_stopCounter(MOTOR_PID_INST);
    motor_pid_reset();
    g_test_left_pwm = 0;
    g_test_right_pwm = 0;
    g_target_speed_left = 0;
    g_target_speed_right = 0;
}

static void show_mode(uint8_t mode)
{
    OLED_Clear();
    switch (mode) {
    case 0:
        OLED_ShowString(0, 0, "Mode 0: STOP", 16);
        break;
    case 1:
        OLED_ShowString(0, 0, "Mode 1: FWD", 16);
        OLED_ShowString(0, 48, "PWM open loop", 12);
        break;
    case 2:
        OLED_ShowString(0, 0, "Mode 2: REV", 16);
        OLED_ShowString(0, 48, "PWM open loop", 12);
        break;
    case 3:
        OLED_ShowString(0, 0, "Mode 3: PID", 16);
        OLED_ShowString(0, 48, "Target speed", 12);
        break;
    default:
        OLED_ShowString(0, 0, "Mode ?", 16);
        break;
    }
}

int main(void)
{
    uint8_t mode = 0U;

    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    NVIC_EnableIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);

    motor_init(1U);
    motor_init(2U);
    reset_motion_state();
    show_mode(mode);
    OLED_ShowString(0, 48, "PB21 to switch", 12);
    OLED_Refresh();

    while (1) {
        if (click()) {
            mode = (uint8_t) ((mode + 1U) % 4U);
            reset_motion_state();

            if (mode == 1U) {
                g_test_left_pwm = 1800;
                g_test_right_pwm = 1800;
                motor_stop();
            } else if (mode == 2U) {
                g_test_left_pwm = -1800;
                g_test_right_pwm = -1800;
                motor_stop();
            } else if (mode == 3U) {
                g_target_speed_left = 120;
                g_target_speed_right = 120;
                target_speed_1 = (float)g_target_speed_left;
                target_speed_2 = (float)g_target_speed_right;
                PWM_1_duty = 0;
                PWM_2_duty = 0;
                DL_Timer_startCounter(MOTOR_PID_INST);
            }

            if (mode == 1U || mode == 2U) {
                motor_set_pwm_lr(g_test_left_pwm, g_test_right_pwm);
            } else if (mode == 3U) {
                motor_stop();
            }

            show_mode(mode);
            OLED_ShowString(0, 48, "PB21 to switch", 12);
            OLED_Refresh();
            delay_ms(80U);
        }

        OLED_ShowString(0, 14, "L:", 12);
        OLED_ShowString(16, 14, (counter_1_A < 0) ? "-" : "+", 12);
        OLED_ShowNum(24, 14, (uint32_t)((counter_1_A < 0) ? -counter_1_A : counter_1_A), 5, 12);
        OLED_ShowString(56, 14, "R:", 12);
        OLED_ShowString(72, 14, (counter_2_A < 0) ? "-" : "+", 12);
        OLED_ShowNum(80, 14, (uint32_t)((counter_2_A < 0) ? -counter_2_A : counter_2_A), 5, 12);

        OLED_ShowString(0, 28, "PWM:", 12);
        if (mode == 3U) {
            OLED_ShowString(32, 28, (PWM_1_duty < 0) ? "-" : "+", 12);
            OLED_ShowNum(40, 28, (uint32_t)((PWM_1_duty < 0) ? -PWM_1_duty : PWM_1_duty), 4, 12);
        } else {
            OLED_ShowString(32, 28, (g_test_left_pwm < 0) ? "-" : "+", 12);
            OLED_ShowNum(40, 28, (uint32_t)((g_test_left_pwm < 0) ? -g_test_left_pwm : g_test_left_pwm), 4, 12);
        }
        OLED_ShowString(64, 28, "/", 12);
        if (mode == 3U) {
            OLED_ShowString(72, 28, (PWM_2_duty < 0) ? "-" : "+", 12);
            OLED_ShowNum(80, 28, (uint32_t)((PWM_2_duty < 0) ? -PWM_2_duty : PWM_2_duty), 4, 12);
        } else {
            OLED_ShowString(72, 28, (g_test_right_pwm < 0) ? "-" : "+", 12);
            OLED_ShowNum(80, 28, (uint32_t)((g_test_right_pwm < 0) ? -g_test_right_pwm : g_test_right_pwm), 4, 12);
        }

        if (mode == 3U) {
            OLED_ShowString(0, 42, "TS:", 12);
            OLED_ShowNum(24, 42, (uint32_t)g_target_speed_left, 3, 12);
            OLED_ShowString(56, 42, "/", 12);
            OLED_ShowNum(64, 42, (uint32_t)g_target_speed_right, 3, 12);
            OLED_ShowString(96, 42, "mm/s", 12);

            OLED_ShowString(0, 54, "Sp:", 12);
            OLED_ShowNum(24, 54, (uint32_t)((speed_1 < 0.0f) ? -speed_1 : speed_1), 3, 12);
            OLED_ShowString(56, 54, "/", 12);
            OLED_ShowNum(64, 54, (uint32_t)((speed_2 < 0.0f) ? -speed_2 : speed_2), 3, 12);
        }

        OLED_Refresh();
        delay_ms(100U);
    }
}
