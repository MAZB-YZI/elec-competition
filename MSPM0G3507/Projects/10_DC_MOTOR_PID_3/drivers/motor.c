#include "motor.h"

void motor_init(uint8_t motor_id)
{
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    if(motor_id == 1){
        DL_Timer_startCounter(PWMA_INST);
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMA_INST, 0, GPIO_PWMA_C0_IDX);
    }
    else if(motor_id == 2){
        // DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        // DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
    }
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    if(duty > 4000){
        duty = 4000;
    }
    if(motor_id == 1){
        DL_Timer_setCaptureCompareValue(PWMA_INST, duty, GPIO_PWMA_C0_IDX);
    }
    else if(motor_id == 2){
        // DL_Timer_setCaptureCompareValue(PWMB_INST, speed, GPIO_PWMB_C0_IDX);
    }
}

// direction: 0 停止，1 正转，2 反转
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == 2){
        // if(direction == 0){
        //     DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        //     DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        // }
        // else if(direction == 1){
        //     DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        //     DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        // }
        // else if(direction == 2){
        //     DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        //     DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        // }
    }
}


extern uint32_t counter_1_A;
float speed_1 = 0;
float speed_2 = 0;

void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20; // 轮速 mm/s
        counter_1_A = 0; // 计算完速度后清零计数器
    }
    if (motor_id == 2) {
        // speed_2 = (float)counter_1_B / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20; // 轮速 mm/s
        // counter_1_B = 0; // 计算完速度后清零计数器
    }
}

float kp = 0.5; // 比例系数
float ki = 0.4; // 积分系数

uint16_t PWM_1_duty = 0;
float target_speed_1 = 0; // 目标速度 mm/s
// float target_speed_2 = 0; // 目标速度 mm/s
float last_error_1 = 0;
float current_error_1 = 0;

void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    if (motor_id == 1) {
        error = target_speed_1 - speed_1;
        current_error_1 = error;
        PWM_1_duty += (uint16_t)(kp * (current_error_1-last_error_1) + ki *(current_error_1));
        last_error_1 = current_error_1;
        motor_set_duty(motor_id, PWM_1_duty);
    }
    if (motor_id == 2) {
        // error = target_speed - speed_2;
        // uint32_t duty = (uint32_t)(error * 100);
        // motor_set_duty(motor_id, duty);
    }
}

void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        calculate_speed(1);
        DC_MOTOR_PID(1);
        break;
    // case DL_TIMER_IIDX_COMPARE_0:
    //     status = (status + 3 -1) % 3;
    //     /* code */
    //     break;
    
    default:
        break;
    }
}



