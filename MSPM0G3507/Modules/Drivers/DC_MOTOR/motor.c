/**
 * motor.c — TB6612 双电机驱动 + 编码器 + PID (MSPM0 DriverLib)
 */

#include "motor.h"

/* 编码器脉冲计数 (ISR 中累加) */
volatile int32_t g_enc_left  = 0;
volatile int32_t g_enc_right = 0;

/* ================================================================
 *  电机初始化
 * ================================================================ */
void Motor_Init(void)
{
    Motor_Stop();
    Encoder_ResetCounts();
}

/* ================================================================
 *  TB6612 方向控制 (使用 SysConfig 生成的独立端口宏)
 * ================================================================ */

static void Motor_LeftDir(bool in1, bool in2)
{
    if (in1)
        DL_GPIO_setPins(MOTOR_DIR_L_DIR1_PORT, MOTOR_DIR_L_DIR1_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_L_DIR1_PORT, MOTOR_DIR_L_DIR1_PIN);

    if (in2)
        DL_GPIO_setPins(MOTOR_DIR_L_DIR2_PORT, MOTOR_DIR_L_DIR2_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_L_DIR2_PORT, MOTOR_DIR_L_DIR2_PIN);
}

static void Motor_RightDir(bool in1, bool in2)
{
    if (in1)
        DL_GPIO_setPins(MOTOR_DIR_R_DIR1_PORT, MOTOR_DIR_R_DIR1_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_R_DIR1_PORT, MOTOR_DIR_R_DIR1_PIN);

    if (in2)
        DL_GPIO_setPins(MOTOR_DIR_R_DIR2_PORT, MOTOR_DIR_R_DIR2_PIN);
    else
        DL_GPIO_clearPins(MOTOR_DIR_R_DIR2_PORT, MOTOR_DIR_R_DIR2_PIN);
}

/* ================================================================
 *  电机调速
 * ================================================================ */

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
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        duty, DL_TIMER_CC_0_INDEX);
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
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        duty, DL_TIMER_CC_1_INDEX);
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
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        MOTOR_PWM_MAX, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
        MOTOR_PWM_MAX, DL_TIMER_CC_1_INDEX);
}

/* ================================================================
 *  编码器读取
 * ================================================================ */

int32_t Encoder_GetLeftCount(void)  { return g_enc_left; }
int32_t Encoder_GetRightCount(void) { return g_enc_right; }

void Encoder_ResetCounts(void)
{
    g_enc_left  = 0;
    g_enc_right = 0;
}

/**
 * @brief 编码器 GPIO 中断处理 (GPIOA)
 *
 * 在 GPIOA 中断服务中调用:
 *   void GROUP1_IRQHandler(void) { Encoder_ISR(); }
 *
 * 编码器 A 相上升沿触发, 读 B 相电平判方向:
 *   B=高 → 正转 (++),  B=低 → 反转 (--)
 */
void Encoder_ISR(void)
{
    DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(ENCODER_PORT);
    uint16_t pin;

    /* 编码器 A (左电机): PA27=A相, PA26=B相 */
    if (iidx == ENCODER_ENC_A1_IIDX) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ENC_A1_IIDX);
        if (DL_GPIO_readPins(ENCODER_PORT, ENCODER_ENC_A2_PIN))
            g_enc_left++;
        else
            g_enc_left--;
    }

    /* 编码器 B (右电机): PA14=A相, PA25=B相 */
    else if (iidx == ENCODER_ENC_B1_IIDX) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ENC_B1_IIDX);
        if (DL_GPIO_readPins(ENCODER_PORT, ENCODER_ENC_B2_PIN))
            g_enc_right++;
        else
            g_enc_right--;
    }
}

/* ================================================================
 *  PID 控制器
 * ================================================================ */

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, int16_t out_limit)
{
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit  = (float)out_limit * 0.5f;
    pid->output_limit    = out_limit;
}

int16_t PID_Compute(PID_t *pid, int16_t setpoint, int16_t measurement, float dt)
{
    float error  = (float)(setpoint - measurement);
    float output = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    output += pid->Ki * pid->integral;

    if (dt > 0.001f)
        output += pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return (int16_t)output;
}
