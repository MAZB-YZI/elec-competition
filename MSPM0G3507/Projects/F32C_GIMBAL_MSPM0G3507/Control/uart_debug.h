#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "board.h"
#include <stdint.h>

/* 命令回调函数类型 */
typedef void (*uart_cmd_callback_t)(uint8_t cmd_type, int32_t value);

/* 命令类型定义（DBG_ 前缀避免与 DataScope_DP.h 冲突） */
#define DBG_CMD_NONE        0
#define DBG_CMD_M1_POS      1   /* 电机1绝对位置 */
#define DBG_CMD_M2_POS      2   /* 电机2绝对位置 */
#define DBG_CMD_M1_SPD      3   /* 电机1转速 */
#define DBG_CMD_M2_SPD      4   /* 电机2转速 */
#define DBG_CMD_M1_MODE     5   /* 电机1模式 */
#define DBG_CMD_M2_MODE     6   /* 电机2模式 */
#define DBG_CMD_M1_ADD      7   /* 电机1增加角度 */
#define DBG_CMD_M1_SUB      8   /* 电机1减少角度 */
#define DBG_CMD_M2_ADD      9   /* 电机2增加角度 */
#define DBG_CMD_M2_SUB      10  /* 电机2减少角度 */
#define DBG_CMD_ENABLE      11  /* 使能电机 */
#define DBG_CMD_DISABLE     12  /* 失能电机 */
#define DBG_CMD_SAVE        13  /* 保存参数 */
#define DBG_CMD_ZERO        14  /* 清零位置 */
#define DBG_CMD_STATUS      15  /* 请求状态 */
#define DBG_CMD_SCAN        16  /* 扫描电机ID */

/* 函数声明 */
void UART_Debug_Init(void);
void UART_Debug_SendString(const char *str);
void UART_Debug_SendStatus(int m1_target, int m1_current, int m1_speed,
                           int m2_target, int m2_current, int m2_speed);
void UART_Debug_RegisterCallback(uart_cmd_callback_t cb);
void UART_Debug_Process(void);

#endif
