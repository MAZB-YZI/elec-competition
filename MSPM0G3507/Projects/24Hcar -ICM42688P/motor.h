/**
 * motor.h — TB6612 双电机驱动 + PID 速度控制
 *
 * 依赖: SysConfig 生成的 ti_msp_dl_config.h
 * PWM:  TIMG0, CCP0=PA12 (左), CCP1=PA13 (右), timerCount=4000
 * DIR:  PB19(L_DIR1), PB17(L_DIR2), PA16(R_DIR1), PB24(R_DIR2)
 * ENC:  PA27/PA26 (编码器A, 左), PA14/PA25 (编码器B, 右)
 *
 * TB6612 真值表:
 *   IN1=H, IN2=L  → 正转 (CW)
 *   IN1=L, IN2=H  → 反转 (CCW)
 *   IN1=L, IN2=L  → 短接刹车
 *   IN1=H, IN2=H  → 停止
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* PWM 参数 (与 SysConfig timerCount 一致) */
#define MOTOR_PWM_MAX     4000
#define MOTOR_PWM_MIN    -4000

/* 编码器线数 (每圈脉冲数) */
#define ENCODER_PPR         11
#define GEAR_RATIO          30
#define PULSES_PER_REV      (ENCODER_PPR * GEAR_RATIO)  /* 每圈 330 脉冲 */

/* ---------- 电机接口 ---------- */

void Motor_Init(void);
void Motor_SetLeftSpeed(int16_t speed);
void Motor_SetRightSpeed(int16_t speed);
void Motor_Stop(void);
void Motor_Brake(void);

/* ---------- 编码器 ---------- */

int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_ResetCounts(void);
void Encoder_ResetDistance(void);
float Encoder_GetLeftDistanceCm(void);
float Encoder_GetRightDistanceCm(void);
float Encoder_GetAverageDistanceCm(void);
void Encoder_ISR(void);

/* ---------- PID 控制器 ---------- */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float integral_limit;
    int16_t output_limit;
} PID_t;

void  PID_Init(PID_t *pid, float Kp, float Ki, float Kd, int16_t out_limit);
void  PID_Reset(PID_t *pid);
int16_t PID_Compute(PID_t *pid, int16_t setpoint, int16_t measurement, float dt);

#endif /* __MOTOR_H__ */
