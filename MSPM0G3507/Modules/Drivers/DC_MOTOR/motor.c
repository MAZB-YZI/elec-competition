#include "motor.h"

volatile int32_t g_enc_left  = 0;
volatile int32_t g_enc_right = 0;

void Motor_Init(void)
{
    Motor_Stop();
    Encoder_ResetCounts();
}

static void Motor_LeftDir(bool in1, bool in2)
{
    if (in1) {
        DL_GPIO_setPins(MOTOR_DIR_L_DIR1_PORT, MOTOR_DIR_L_DIR1_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_L_DIR1_PORT, MOTOR_DIR_L_DIR1_PIN);
    }

    if (in2) {
        DL_GPIO_setPins(MOTOR_DIR_L_DIR2_PORT, MOTOR_DIR_L_DIR2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_L_DIR2_PORT, MOTOR_DIR_L_DIR2_PIN);
    }
}

static void Motor_RightDir(bool in1, bool in2)
{
    if (in2) {
        DL_GPIO_setPins(MOTOR_DIR_R_DIR1_PORT, MOTOR_DIR_R_DIR1_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_R_DIR1_PORT, MOTOR_DIR_R_DIR1_PIN);
    }

    if (in1) {
        DL_GPIO_setPins(MOTOR_DIR_R_DIR2_PORT, MOTOR_DIR_R_DIR2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_R_DIR2_PORT, MOTOR_DIR_R_DIR2_PIN);
    }
}

void Motor_SetLeftSpeed(int16_t speed)
{
    uint16_t duty;

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

int32_t Encoder_GetLeftCount(void)
{
    return g_enc_left;
}

int32_t Encoder_GetRightCount(void)
{
    return g_enc_right;
}

void Encoder_ResetCounts(void)
{
    g_enc_left = 0;
    g_enc_right = 0;
}

void Encoder_ISR(void)
{
    DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(ENCODER_PORT);

    if (iidx == ENCODER_ENC_A1_IIDX) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ENC_A1_IIDX);
        if (DL_GPIO_readPins(ENCODER_PORT, ENCODER_ENC_A2_PIN)) {
            g_enc_left++;
        } else {
            g_enc_left--;
        }
    } else if (iidx == ENCODER_ENC_B1_IIDX) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ENC_B1_IIDX);
        if (DL_GPIO_readPins(ENCODER_PORT, ENCODER_ENC_B2_PIN)) {
            g_enc_right++;
        } else {
            g_enc_right--;
        }
    }
}
