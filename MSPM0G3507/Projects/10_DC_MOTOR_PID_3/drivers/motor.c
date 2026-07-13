#include "motor.h"

extern volatile int32_t counter_1_A;
extern volatile int32_t counter_2_A;

static uint8_t g_pwm_started = 0U;
static uint8_t g_pid_started = 0U;

static void motor_apply_compare(uint8_t motor_id, uint32_t duty)
{
    if (motor_id == 1U) {
        DL_Timer_setCaptureCompareValue(PWMA_INST, duty, GPIO_PWMA_C0_IDX);
    } else if (motor_id == 2U) {
        DL_Timer_setCaptureCompareValue(PWMA_INST, duty, GPIO_PWMA_C1_IDX);
    }
}

void motor_init(uint8_t motor_id)
{
    (void) motor_id;

    if (g_pwm_started == 0U) {
        DL_Timer_startCounter(PWMA_INST);
        g_pwm_started = 1U;
    }

    if (g_pid_started == 0U) {
        DL_Timer_startCounter(MOTOR_PID_INST);
        NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
        g_pid_started = 1U;
    }

    motor_apply_compare(1U, 0U);
    motor_apply_compare(2U, 0U);
    motor_stop();
}

int32_t motor_limit_pwm(int32_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) {
        pwm = MOTOR_PWM_MAX;
    }
    if (pwm < -MOTOR_PWM_MAX) {
        pwm = -MOTOR_PWM_MAX;
    }
    return pwm;
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    if (duty > (uint32_t) MOTOR_PWM_MAX) {
        duty = (uint32_t) MOTOR_PWM_MAX;
    }
    motor_apply_compare(motor_id, duty);
}

void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == 1U) {
        if (direction == 0U) {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == 1U) {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == 2U) {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == 3U) {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    } else if (motor_id == 2U) {
        if (direction == 0U) {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == 1U) {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == 2U) {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == 3U) {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

void motor_set_pwm(uint8_t motor_id, int32_t pwm)
{
    uint32_t duty;

    pwm = motor_limit_pwm(pwm);

    if (pwm > 0) {
        duty = (uint32_t) pwm;
        if (motor_id == 2U) {
            motor_set_direction(motor_id, 2U);
        } else {
            motor_set_direction(motor_id, 1U);
        }
        motor_set_duty(motor_id, duty);
    } else if (pwm < 0) {
        duty = (uint32_t) (-pwm);
        if (motor_id == 2U) {
            motor_set_direction(motor_id, 1U);
        } else {
            motor_set_direction(motor_id, 2U);
        }
        motor_set_duty(motor_id, duty);
    } else {
        motor_coast(motor_id);
    }
}

void motor_set_pwm_lr(int32_t left_pwm, int32_t right_pwm)
{
    motor_set_pwm(1U, left_pwm);
    motor_set_pwm(2U, right_pwm);
}

void motor_coast(uint8_t motor_id)
{
    motor_set_duty(motor_id, 0U);
    motor_set_direction(motor_id, 0U);
}

void motor_coast_all(void)
{
    motor_coast(1U);
    motor_coast(2U);
}

void motor_brake(uint8_t motor_id)
{
    motor_set_duty(motor_id, 0U);
    motor_set_direction(motor_id, 3U);
}

void motor_brake_all(void)
{
    motor_brake(1U);
    motor_brake(2U);
}

void motor_stop(void)
{
    motor_coast_all();
}

void motor_pid_reset(void)
{
    counter_1_A = 0;
    counter_2_A = 0;
    motor_stop();
}

float speed_1 = 0.0f;
float speed_2 = 0.0f;

static void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1U) {
        speed_1 = ((float) counter_1_A / MOTOR_BIANMAQI) * PI * MOTOR_WHEEL_D * 100.0f;
        counter_1_A = 0;
    }
    if (motor_id == 2U) {
        speed_2 = ((float) counter_2_A / MOTOR_BIANMAQI) * PI * MOTOR_WHEEL_D * 100.0f;
        counter_2_A = 0;
    }
}

void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        calculate_speed(1U);
        calculate_speed(2U);
        break;
    default:
        break;
    }
}
