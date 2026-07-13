#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "../../../Modules/Control/pid.h"

#define PI                 3.14f
#define MOTOR_BIANMAQI     260
#define MOTOR_WHEEL_D      67
#define MOTOR_PWM_MAX      4000
#define MOTOR_PID_PWM_MAX  1800

/*
 * TB6612 wiring used by this test project:
 *   Motor1 PWM : PA12
 *   Motor1 DIR : PB19 / PB17
 *   Motor2 PWM : PA13
 *   Motor2 DIR : PA16 / PB24
 */

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

int32_t motor_limit_pwm(int32_t pwm);
void motor_set_pwm(uint8_t motor_id, int32_t pwm);
void motor_set_pwm_lr(int32_t left_pwm, int32_t right_pwm);
void motor_coast(uint8_t motor_id);
void motor_coast_all(void);
void motor_brake(uint8_t motor_id);
void motor_brake_all(void);
void motor_stop(void);
void motor_pid_reset(void);
void Encoder_ISR(void);

#endif // MOTOR_H
