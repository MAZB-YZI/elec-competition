#ifndef __TB6612_H
#define __TB6612_H

#include "board.h"

/* TB6612 引脚定义（需要根据实际接线修改） */
#define TB6612_AIN1_PORT    GPIOA
#define TB6612_AIN1_PIN     DL_GPIO_PIN_0
#define TB6612_AIN2_PORT    GPIOA
#define TB6612_AIN2_PIN     DL_GPIO_PIN_1
#define TB6612_PWMA_PORT    GPIOA
#define TB6612_PWMA_PIN     DL_GPIO_PIN_2

#define TB6612_BIN1_PORT    GPIOA
#define TB6612_BIN1_PIN     DL_GPIO_PIN_3
#define TB6612_BIN2_PORT    GPIOA
#define TB6612_BIN2_PIN     DL_GPIO_PIN_4
#define TB6612_PWMB_PORT    GPIOA
#define TB6612_PWMB_PIN     DL_GPIO_PIN_5

/* 电机选择 */
#define MOTOR_A             0
#define MOTOR_B             1

/* 电机方向 */
#define MOTOR_FORWARD       0
#define MOTOR_BACKWARD      1
#define MOTOR_STOP          2

/* 函数声明 */
void TB6612_Init(void);
void TB6612_SetMotor(uint8_t motor, uint8_t direction, uint16_t speed);
void TB6612_Stop(uint8_t motor);
void TB6612_StopAll(void);

#endif /* __TB6612_H */
