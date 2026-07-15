/*
 * MSPM0G3507 F32C TTL 无刷云台电机位置闭环控制
 * TTL 通信使用 UART3 (PB2/PB3)
 * 上位机调试使用 UART0 (PA10/PA11)
 */
#include "board.h"
#include "uart_callback.h"
#include "DataScope_DP.h"
#include "uart_debug.h"
#include "show.h"
#include <string.h>

/* 电机参数 */
int Motor1_Speed = 10, Motor2_Speed = 10;
int motor1_Current_Speed, motor2_Current_Speed;
int Motor1_T_Position = 0, Motor2_T_Position = 0;
int Motor1_Current_Position, Motor2_Current_Position;

uint8_t motor1_ID = 1, motor2_ID = 2;  /* YAW=1, PITCH=2 */
uint8_t motor1_Mode = 1, motor2_Mode = 1;  /* 1 = 多圈位置闭环模式 */

/* 上电软件零点 */
int32_t motor1_zero_offset = 0;
int32_t motor2_zero_offset = 0;
volatile uint8_t motor1_position_valid = 0;
volatile uint8_t motor2_position_valid = 0;

/*
 * 上电设当前位置为零点
 * 返回 1=成功，0=超时
 */
uint8_t Gimbal_SetPowerOnZero(void)
{
    motor1_position_valid = 0;
    motor2_position_valid = 0;

    BLDC_ReqFeedback(motor1_ID, FB_MULTI_ANGLE);
    BLDC_ReqFeedback(motor2_ID, FB_MULTI_ANGLE);

    for (uint16_t i = 0; i < 200; i++) {
        if (motor1_position_valid && motor2_position_valid) {
            motor1_zero_offset = Motor1_Current_Position;
            motor2_zero_offset = Motor2_Current_Position;
            return 1;
        }
        delay_ms(10);
    }
    return 0;  /* 超时 */
}

/*
 * 设置相对角度（单位：度）
 * 内部自动乘以10转成0.1度，存储为相对位置
 */
void Gimbal_SetRelativeAngle(float motor1_deg, float motor2_deg)
{
    Motor1_T_Position = (int32_t)(motor1_deg);
    Motor2_T_Position = (int32_t)(motor2_deg);
}

/* 上位机命令回调（value 单位：度，内部也用度） */
static void on_uart_cmd(uint8_t cmd_type, int32_t value)
{
    switch (cmd_type) {
    case DBG_CMD_M1_POS:
        Motor1_T_Position = value;  /* 相对位置 */
        break;
    case DBG_CMD_M2_POS:
        Motor2_T_Position = value;  /* 相对位置 */
        break;
    case DBG_CMD_M1_SPD:
        Motor1_Speed = value;
        BLDC_SetSpeed(motor1_ID, (int16_t)Motor1_Speed);
        break;
    case DBG_CMD_M2_SPD:
        Motor2_Speed = value;
        BLDC_SetSpeed(motor2_ID, (int16_t)Motor2_Speed);
        break;
    case DBG_CMD_M1_MODE:
        motor1_Mode = value;
        BLDC_SetMode(motor1_ID, motor1_Mode);
        break;
    case DBG_CMD_M2_MODE:
        motor2_Mode = value;
        BLDC_SetMode(motor2_ID, motor2_Mode);
        break;
    case DBG_CMD_M1_ADD:
        Motor1_T_Position += value;
        break;
    case DBG_CMD_M1_SUB:
        Motor1_T_Position -= value;
        break;
    case DBG_CMD_M2_ADD:
        Motor2_T_Position += value;
        break;
    case DBG_CMD_M2_SUB:
        Motor2_T_Position -= value;
        break;
    case DBG_CMD_ENABLE:
        BLDC_Enable(motor1_ID);
        delay_ms(1);
        BLDC_Enable(motor2_ID);
        break;
    case DBG_CMD_DISABLE:
        BLDC_Disable(motor1_ID);
        delay_ms(1);
        BLDC_Disable(motor2_ID);
        break;
    case DBG_CMD_SAVE:
        /* 先设置硬件零点，再保存参数到 EEPROM */
        BLDC_SetSingleAngleZero(motor1_ID);
        delay_ms(10);
        BLDC_SetSingleAngleZero(motor2_ID);
        delay_ms(10);
        BLDC_SaveParams(motor1_ID);
        delay_ms(10);
        BLDC_SaveParams(motor2_ID);
        break;
    case DBG_CMD_ZERO:
        /* 先请求位置反馈，等待更新后再设置零点 */
        BLDC_ReqFeedback(motor1_ID, FB_MULTI_ANGLE);
        delay_ms(100);
        BLDC_ReqFeedback(motor2_ID, FB_MULTI_ANGLE);
        delay_ms(100);
        /* 设置零点 */
        motor1_zero_offset = Motor1_Current_Position;
        motor2_zero_offset = Motor2_Current_Position;
        Motor1_T_Position = 0;
        Motor2_T_Position = 0;
        break;
    case DBG_CMD_HOME:
        /* 回到零位（主循环会持续发送位置命令） */
        Motor1_T_Position = 0;
        Motor2_T_Position = 0;
        break;
    case DBG_CMD_STATUS:
        UART_Debug_SendStatus(Motor1_T_Position,
                              Motor1_Current_Position - motor1_zero_offset,
                              Motor1_Speed,
                              motor1_Current_Speed,
                              Motor2_T_Position,
                              Motor2_Current_Position - motor2_zero_offset,
                              Motor2_Speed,
                              motor2_Current_Speed);
        break;
    default:
        break;
    }
}

int main(void)
{
    /* SysConfig 初始化系统时钟、UART、定时器、OLED 引脚等 */
    SYSCFG_DL_init();

    /* 配置 UART3 上拉 */
    DL_GPIO_setDigitalInternalResistor(GPIO_UART_1_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);

    /* 使能 UART3 RX 中断（电机通信） */
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    /* 使能 TIMER0 定时中断 */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* 初始化外设 */
    //OLED_Init();  /* 临时跳过，I2C未接/OLED未响应会卡死 */
    UART_Debug_Init();
    UART_Debug_RegisterCallback(on_uart_cmd);

    /* 同步字节，等待电机上电 */
    usart1_send(0x00);
    delay_ms(5000);  /* 等待 5 秒，让电机充分初始化 */

    /* 1. 使能电机 */
    BLDC_Enable(motor1_ID);
    delay_ms(100);
    BLDC_Enable(motor2_ID);
    delay_ms(100);

    /* 2. 设置多圈位置闭环模式 */
    BLDC_SetMode(motor1_ID, motor1_Mode);
    delay_ms(100);
    BLDC_SetMode(motor2_ID, motor2_Mode);
    delay_ms(100);

    /* 3. 设置位置模式下的速度 */
    BLDC_SetSpeed(motor1_ID, (int16_t)Motor1_Speed);
    delay_ms(100);
    BLDC_SetSpeed(motor2_ID, (int16_t)Motor2_Speed);
    delay_ms(100);

    /* 4. 上电自动设零点（读取电机当前位置作为软件零点） */
    if (Gimbal_SetPowerOnZero()) {
        UART_Debug_SendString("Zero point set OK\r\n");
    } else {
        UART_Debug_SendString("Zero point timeout!\r\n");
    }
    UART_Debug_SendString("Ready\r\n");

    while (1)
    {
        /* 处理上位机命令 */
        UART_Debug_Process();

        /* 发送目标位置（相对零点 + 零点偏移 = 绝对位置） */
        BLDC_SetMultiAngle(motor1_ID, (Motor1_T_Position + motor1_zero_offset) * 10);
        delay_ms(5);
        BLDC_SetMultiAngle(motor2_ID, (Motor2_T_Position + motor2_zero_offset) * 10);
        delay_ms(5);

        /* 请求位置反馈 */
        BLDC_ReqFeedback(motor1_ID, FB_MULTI_ANGLE);
        delay_ms(5);
        BLDC_ReqFeedback(motor2_ID, FB_MULTI_ANGLE);
        delay_ms(5);

        /* 请求速度反馈 */
        BLDC_ReqFeedback(motor1_ID, FB_SPEED);
        delay_ms(5);
        BLDC_ReqFeedback(motor2_ID, FB_SPEED);
        delay_ms(5);

        /* 更新 OLED 显示 */
        //OLED_Show();  /* 临时跳过 */
    }
}
