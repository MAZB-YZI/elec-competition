#include "tb6612.h"

/* 初始化 TB6612 */
void TB6612_Init(void)
{
    /* 配置 GPIO 引脚为输出 */
    DL_GPIO_initDigitalOutput(TB6612_AIN1_PIN);
    DL_GPIO_initDigitalOutput(TB6612_AIN2_PIN);
    DL_GPIO_initDigitalOutput(TB6612_PWMA_PIN);
    DL_GPIO_initDigitalOutput(TB6612_BIN1_PIN);
    DL_GPIO_initDigitalOutput(TB6612_BIN2_PIN);
    DL_GPIO_initDigitalOutput(TB6612_PWMB_PIN);

    /* 初始状态：停止 */
    TB6612_StopAll();
}

/* 设置电机方向和速度 */
void TB6612_SetMotor(uint8_t motor, uint8_t direction, uint16_t speed)
{
    if (motor == MOTOR_A) {
        switch (direction) {
            case MOTOR_FORWARD:
                DL_GPIO_setPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
                DL_GPIO_clearPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
                break;
            case MOTOR_BACKWARD:
                DL_GPIO_clearPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
                DL_GPIO_setPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
                break;
            case MOTOR_STOP:
                DL_GPIO_clearPins(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
                DL_GPIO_clearPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
                break;
        }
        /* PWM 控制速度（需要配置定时器） */
        /* TODO: 实现 PWM 控制 */
    } else if (motor == MOTOR_B) {
        switch (direction) {
            case MOTOR_FORWARD:
                DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
                DL_GPIO_clearPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
                break;
            case MOTOR_BACKWARD:
                DL_GPIO_clearPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
                DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
                break;
            case MOTOR_STOP:
                DL_GPIO_clearPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
                DL_GPIO_clearPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
                break;
        }
        /* PWM 控制速度（需要配置定时器） */
        /* TODO: 实现 PWM 控制 */
    }
}

/* 停止单个电机 */
void TB6612_Stop(uint8_t motor)
{
    TB6612_SetMotor(motor, MOTOR_STOP, 0);
}

/* 停止所有电机 */
void TB6612_StopAll(void)
{
    TB6612_Stop(MOTOR_A);
    TB6612_Stop(MOTOR_B);
}
