#include "motor.h"

#ifndef MOTOR_DEFAULT_PWM_PERIOD
#define MOTOR_DEFAULT_PWM_PERIOD 4000U
#endif

static uint16_t motor_pwm_period = MOTOR_DEFAULT_PWM_PERIOD;

static uint16_t clamp_magnitude(int32_t pwm)
{
    int32_t limit = (int32_t) motor_pwm_period;

    if (pwm > limit) {
        pwm = limit;
    } else if (pwm < -limit) {
        pwm = -limit;
    }

    return (uint16_t) ((pwm < 0) ? -pwm : pwm);
}

static void set_one(MotorId motor, int32_t pwm)
{
    if (pwm > 0) {
        Motor_PlatformSetDirection(motor, MOTOR_DIR_FORWARD);
    } else if (pwm < 0) {
        Motor_PlatformSetDirection(motor, MOTOR_DIR_REVERSE);
    } else {
        Motor_PlatformSetDirection(motor, MOTOR_DIR_COAST);
    }

    Motor_PlatformSetDuty(motor, clamp_magnitude(pwm));
}

void Motor_Init(const MotorConfig *config)
{
    if ((config != 0) && (config->pwm_period > 0U)) {
        motor_pwm_period = config->pwm_period;
    } else {
        motor_pwm_period = MOTOR_DEFAULT_PWM_PERIOD;
    }

    Motor_Stop();
}

void Motor_SetPWM(int32_t left_pwm, int32_t right_pwm)
{
    set_one(MOTOR_LEFT, left_pwm);
    set_one(MOTOR_RIGHT, right_pwm);
}

void Motor_Coast(void)
{
    Motor_PlatformSetDuty(MOTOR_LEFT, 0U);
    Motor_PlatformSetDuty(MOTOR_RIGHT, 0U);
    Motor_PlatformSetDirection(MOTOR_LEFT, MOTOR_DIR_COAST);
    Motor_PlatformSetDirection(MOTOR_RIGHT, MOTOR_DIR_COAST);
}

void Motor_Brake(void)
{
    Motor_PlatformSetDuty(MOTOR_LEFT, 0U);
    Motor_PlatformSetDuty(MOTOR_RIGHT, 0U);
    Motor_PlatformSetDirection(MOTOR_LEFT, MOTOR_DIR_BRAKE);
    Motor_PlatformSetDirection(MOTOR_RIGHT, MOTOR_DIR_BRAKE);
}

void Motor_Stop(void)
{
    Motor_Coast();
}

uint16_t Motor_GetPwmPeriod(void)
{
    return motor_pwm_period;
}
