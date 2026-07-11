#include "motor.h"
#include "hcar_hal.h"

static uint16_t magnitude(int32_t value)
{
    int32_t limit = (int32_t)HCarHal_GetMotorPeriod();
    if (value > limit) value = limit;
    if (value < -limit) value = -limit;
    return (uint16_t)(value < 0 ? -value : value);
}

static void set_one(bool left, int32_t pwm)
{
    if (pwm > 0) HCarHal_SetMotorDirection(left, true, false);
    else if (pwm < 0) HCarHal_SetMotorDirection(left, false, true);
    else HCarHal_SetMotorDirection(left, false, false);
    HCarHal_SetMotorDuty(left, magnitude(pwm));
}

void Motor_Init(void) { Motor_Stop(); }
void Motor_SetPWM(int32_t left_pwm, int32_t right_pwm)
{ set_one(true, left_pwm); set_one(false, right_pwm); }
void Motor_Coast(void)
{ HCarHal_SetMotorDuty(true, 0); HCarHal_SetMotorDuty(false, 0);
  HCarHal_SetMotorDirection(true, false, false); HCarHal_SetMotorDirection(false, false, false); }
void Motor_Brake(void)
{ HCarHal_SetMotorDuty(true, 0); HCarHal_SetMotorDuty(false, 0);
  HCarHal_SetMotorDirection(true, true, true); HCarHal_SetMotorDirection(false, true, true); }
void Motor_Stop(void) { Motor_Coast(); }
