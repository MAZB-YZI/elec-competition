#include "motor.h"
#include <assert.h>
#include <stdint.h>

static MotorDirection direction[2];
static uint16_t duty[2];

void Motor_PlatformSetDirection(MotorId motor, MotorDirection value)
{
    direction[motor] = value;
}

void Motor_PlatformSetDuty(MotorId motor, uint16_t value)
{
    duty[motor] = value;
}

int main(void)
{
    MotorConfig config = { .pwm_period = 1000U };

    Motor_Init(&config);
    assert(Motor_GetPwmPeriod() == 1000U);
    assert(direction[MOTOR_LEFT] == MOTOR_DIR_COAST);
    assert(direction[MOTOR_RIGHT] == MOTOR_DIR_COAST);
    assert(duty[MOTOR_LEFT] == 0U);
    assert(duty[MOTOR_RIGHT] == 0U);

    Motor_SetPWM(500, -1200);
    assert(direction[MOTOR_LEFT] == MOTOR_DIR_FORWARD);
    assert(duty[MOTOR_LEFT] == 500U);
    assert(direction[MOTOR_RIGHT] == MOTOR_DIR_REVERSE);
    assert(duty[MOTOR_RIGHT] == 1000U);

    Motor_SetPWM(0, 0);
    assert(direction[MOTOR_LEFT] == MOTOR_DIR_COAST);
    assert(direction[MOTOR_RIGHT] == MOTOR_DIR_COAST);
    assert(duty[MOTOR_LEFT] == 0U);
    assert(duty[MOTOR_RIGHT] == 0U);

    Motor_Brake();
    assert(direction[MOTOR_LEFT] == MOTOR_DIR_BRAKE);
    assert(direction[MOTOR_RIGHT] == MOTOR_DIR_BRAKE);
    assert(duty[MOTOR_LEFT] == 0U);
    assert(duty[MOTOR_RIGHT] == 0U);

    return 0;
}
