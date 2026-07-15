#ifndef HCAR_MOTOR_H
#define HCAR_MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetLeftSpeed(int16_t speed);
void Motor_SetRightSpeed(int16_t speed);
void Motor_Stop(void);
void Motor_Brake(void);

#endif
