#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorId;

typedef enum {
    MOTOR_DIR_COAST = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_BRAKE
} MotorDirection;

typedef struct {
    uint16_t pwm_period;
} MotorConfig;

void Motor_Init(const MotorConfig *config);
void Motor_SetPWM(int32_t left_pwm, int32_t right_pwm);
void Motor_Coast(void);
void Motor_Brake(void);
void Motor_Stop(void);
uint16_t Motor_GetPwmPeriod(void);

void Motor_PlatformSetDirection(MotorId motor, MotorDirection direction);
void Motor_PlatformSetDuty(MotorId motor, uint16_t duty);

#endif
