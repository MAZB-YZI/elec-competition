#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "../../Control/pid.h"

#define MOTOR_PWM_MAX     4000
#define MOTOR_PWM_MIN    -4000

#define ENCODER_PPR         11
#define GEAR_RATIO          30
#define PULSES_PER_REV      (ENCODER_PPR * GEAR_RATIO)

void Motor_Init(void);
void Motor_SetLeftSpeed(int16_t speed);
void Motor_SetRightSpeed(int16_t speed);
void Motor_Stop(void);
void Motor_Brake(void);

int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_ResetCounts(void);
void Encoder_ISR(void);

#endif /* __MOTOR_H__ */
