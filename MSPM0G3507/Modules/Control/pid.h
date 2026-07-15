#ifndef MODULES_CONTROL_PID_H
#define MODULES_CONTROL_PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float integral_limit;
    int16_t output_limit;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, int16_t out_limit);
void PID_Reset(PID_t *pid);
int16_t PID_Compute(PID_t *pid, int16_t setpoint, int16_t measurement, float dt);

#endif /* MODULES_CONTROL_PID_H */
