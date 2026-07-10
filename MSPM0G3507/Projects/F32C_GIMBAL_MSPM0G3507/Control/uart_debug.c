#include "uart_debug.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>

/* 接收缓冲区 */
static char rx_buf[64];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_complete = 0;

/* 命令回调 */
static uart_cmd_callback_t cmd_callback = NULL;

/* 注册回调函数 */
void UART_Debug_RegisterCallback(uart_cmd_callback_t cb)
{
    cmd_callback = cb;
}

/* 发送单字节 */
static void uart_debug_send_byte(uint8_t data)
{
    while (DL_UART_isTXFIFOFull(UART_DEBUG_INST));
    DL_UART_Main_transmitData(UART_DEBUG_INST, data);
}

/* 发送字符串 */
void UART_Debug_SendString(const char *str)
{
    while (*str) {
        uart_debug_send_byte(*str++);
    }
}

/* 发送状态反馈（角度单位：度，速度单位：RPM） */
void UART_Debug_SendStatus(int m1_target, int m1_current, int m1_target_speed, int m1_current_speed,
                           int m2_target, int m2_current, int m2_target_speed, int m2_current_speed)
{
    char buf[80];

    sprintf(buf, "YAW:目标=%d°,当前=%d°,目标速度=%dRPM,实时速度=%dRPM\r\n",
            m1_target, m1_current, m1_target_speed, m1_current_speed);
    UART_Debug_SendString(buf);

    sprintf(buf, "PITCH:目标=%d°,当前=%d°,目标速度=%dRPM,实时速度=%dRPM\r\n",
            m2_target, m2_current, m2_target_speed, m2_current_speed);
    UART_Debug_SendString(buf);
}

/* 解析命令 */
static void uart_debug_parse_cmd(char *cmd)
{
    int32_t value = 0;
    uint8_t cmd_type = DBG_CMD_NONE;

    if (strncmp(cmd, "M1POS", 5) == 0) {
        cmd_type = DBG_CMD_M1_POS;
        value = atoi(&cmd[5]);
    } else if (strncmp(cmd, "M2POS", 5) == 0) {
        cmd_type = DBG_CMD_M2_POS;
        value = atoi(&cmd[5]);
    } else if (strncmp(cmd, "M1SPD", 5) == 0) {
        cmd_type = DBG_CMD_M1_SPD;
        value = atoi(&cmd[5]);
    } else if (strncmp(cmd, "M2SPD", 5) == 0) {
        cmd_type = DBG_CMD_M2_SPD;
        value = atoi(&cmd[5]);
    } else if (strncmp(cmd, "M1MODE", 6) == 0) {
        cmd_type = DBG_CMD_M1_MODE;
        value = atoi(&cmd[6]);
    } else if (strncmp(cmd, "M2MODE", 6) == 0) {
        cmd_type = DBG_CMD_M2_MODE;
        value = atoi(&cmd[6]);
    } else if (strncmp(cmd, "M1ADD", 5) == 0) {
        cmd_type = DBG_CMD_M1_ADD;
        value = atoi(&cmd[5]);
    } else if (strncmp(cmd, "M1SUB", 5) == 0) {
        cmd_type = DBG_CMD_M1_SUB;
        value = atoi(&cmd[5]);
    } else if (strncmp(cmd, "M2ADD", 5) == 0) {
        cmd_type = DBG_CMD_M2_ADD;
        value = atoi(&cmd[5]);
    } else if (strncmp(cmd, "M2SUB", 5) == 0) {
        cmd_type = DBG_CMD_M2_SUB;
        value = atoi(&cmd[5]);
    } else if (strcmp(cmd, "ENABLE") == 0) {
        cmd_type = DBG_CMD_ENABLE;
    } else if (strcmp(cmd, "DISABLE") == 0) {
        cmd_type = DBG_CMD_DISABLE;
    } else if (strcmp(cmd, "SAVE") == 0) {
        cmd_type = DBG_CMD_SAVE;
    } else if (strcmp(cmd, "ZERO") == 0) {
        cmd_type = DBG_CMD_ZERO;
    } else if (strcmp(cmd, "STATUS") == 0) {
        cmd_type = DBG_CMD_STATUS;
    } else if (strcmp(cmd, "HOME") == 0) {
        cmd_type = DBG_CMD_HOME;
    }

    if (cmd_type != DBG_CMD_NONE && cmd_callback != NULL) {
        cmd_callback(cmd_type, value);
        UART_Debug_SendString("OK\r\n");
    }
}

/* 主循环调用的处理函数 */
void UART_Debug_Process(void)
{
    if (rx_complete) {
        uart_debug_parse_cmd((char *)rx_buf);
        rx_index = 0;
        rx_complete = 0;
    }
}

/* UART0 RX 中断处理 */
void UART_DEBUG_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST) == DL_UART_MAIN_IIDX_RX) {
        uint8_t ch = DL_UART_Main_receiveData(UART_DEBUG_INST);

        if (ch == '\n' || ch == '\r') {
            if (rx_index > 0) {
                rx_buf[rx_index] = '\0';
                rx_complete = 1;
            }
        } else if (rx_index < sizeof(rx_buf) - 1) {
            rx_buf[rx_index++] = ch;
        }
    }
}

/* 初始化 */
void UART_Debug_Init(void)
{
    rx_index = 0;
    rx_complete = 0;

    /* 使能 UART0 RX 中断 */
    DL_UART_Main_enableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    UART_Debug_SendString("UART Debug Ready\r\n");
}
