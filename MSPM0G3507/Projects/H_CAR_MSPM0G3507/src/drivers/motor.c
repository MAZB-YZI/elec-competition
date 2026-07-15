/**
 * motor.c — H_CAR 板级电机驱动适配
 *
 * 使用 H_CAR SysConfig 生成的宏名，适配 Modules/Drivers/DC_MOTOR 的接口
 * 宏名对照：
 *   H_CAR: MOTOR_DIR_L_PORT, MOTOR_DIR_L_L_DIR1_PIN
 *   模块:  MOTOR_DIR_L_DIR1_PORT, MOTOR_DIR_L_DIR1_PIN
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_PWM_MAX  4000
#define MOTOR_PWM_MIN -4000

/* 前向声明 */
void Motor_Stop(void);

/* ========== 方向控制 ========== */

static void Motor_LeftDir(bool in1, bool in2)
{
    if (in1)
        DL_GPIO_setPins(MOTOR_DIR_L_PORT, MOTOR_DIR_L_L_DIR1_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_L_PORT, MOTOR_DIR_L_L_DIR1_PIN);

    if (in2)
        DL_GPIO_setPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_L_DIR2_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_L_DIR2_PIN);
}

static void Motor_RightDir(bool in1, bool in2)
{
    if (in1)
        DL_GPIO_setPins(MOTOR_DIR_R_PORT, MOTOR_DIR_R_R_DIR1_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_R_PORT, MOTOR_DIR_R_R_DIR1_PIN);

    if (in2)
        DL_GPIO_setPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_R_DIR2_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_R_DIR2_PIN);
}

/* ========== 公共接口 ========== */

void Motor_Init(void)
{
    Motor_Stop();
}

void Motor_SetLeftSpeed(int16_t speed)
{
    uint16_t duty;

    /* 两个电机方向都取反 */
    speed = -speed;

    if (speed > 0) {
        Motor_LeftDir(true, false);
        duty = (uint16_t)speed;
    } else if (speed < 0) {
        Motor_LeftDir(false, true);
        duty = (uint16_t)(-speed);
    } else {
        Motor_LeftDir(false, false);
        duty = 0;
    }

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, duty, DL_TIMER_CC_0_INDEX);
}

void Motor_SetRightSpeed(int16_t speed)
{
    uint16_t duty;

    /* 两个电机方向都取反 */
    speed = -speed;

    if (speed > 0) {
        Motor_RightDir(true, false);
        duty = (uint16_t)speed;
    } else if (speed < 0) {
        Motor_RightDir(false, true);
        duty = (uint16_t)(-speed);
    } else {
        Motor_RightDir(false, false);
        duty = 0;
    }

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, duty, DL_TIMER_CC_1_INDEX);
}

void Motor_Stop(void)
{
    Motor_LeftDir(false, false);
    Motor_RightDir(false, false);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
}

void Motor_Brake(void)
{
    Motor_LeftDir(true, true);
    Motor_RightDir(true, true);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, MOTOR_PWM_MAX, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, MOTOR_PWM_MAX, DL_TIMER_CC_1_INDEX);
}

void Motor_SetPWM(int32_t left, int32_t right)
{
    Motor_SetLeftSpeed((int16_t)left);
    Motor_SetRightSpeed((int16_t)right);
}
