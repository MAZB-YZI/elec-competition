#ifndef HCAR_MOTOR_H
#define HCAR_MOTOR_H
#include <stdint.h>
void Motor_Init(void);
void Motor_SetPWM(int32_t left_pwm, int32_t right_pwm);
void Motor_Coast(void);
void Motor_Brake(void);
void Motor_Stop(void);
#endif
