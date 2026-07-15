#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, int16_t out_limit)
{
    if (pid == 0) {
        return;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = (float)out_limit * 0.5f;
    pid->output_limit = out_limit;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

int16_t PID_Compute(PID_t *pid, int16_t setpoint, int16_t measurement, float dt)
{
    float error;
    float output;

    if (pid == 0) {
        return 0;
    }

    error = (float)(setpoint - measurement);
    output = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    output += pid->Ki * pid->integral;

    if (dt > 0.001f) {
        output += pid->Kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return (int16_t)output;
}
