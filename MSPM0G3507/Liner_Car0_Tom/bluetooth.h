/**
 * bluetooth.h — HC-05 蓝牙串口调参 + 连接状态检测
 *
 * 检测方式（二选一或互补）:
 *   1. 软件超时: 超过 BT_TIMEOUT_MS 没收到数据则认为断开
 *   2. STATE 引脚: HC-05 STATE 脚高电平=已连接（需接线）
 *
 * STATE 引脚接线 (可选):
 *   将 HC-05 的 STATE 引脚接到 MSPM0G3507 的 PB14 (GPIO_POOL)
 *   然后取消下面 #define BT_USE_STATE_PIN 的注释即可
 */

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "ti_msp_dl_config.h"
#include <stdbool.h>

/* ---- 配置 ---- */
#define BT_TIMEOUT_MS    3000      /* 超时判定断开 (ms) */

// #define BT_USE_STATE_PIN           /* 启用 STATE 引脚检测（需接线） */
#define BT_STATE_PORT   GPIO_GPIO_POOL_PORT       /* STATE 引脚所在端口 */
#define BT_STATE_PIN    GPIO_GPIO_POOL_GPIO_PB14_PIN  /* PB14 → HC-05 STATE */

/* 可调参数 */
typedef struct {
    float   KP;
    float   KI;
    int16_t BASE_PWM;
    int16_t TURN_SPEED;
    int16_t OUTPUT_LIM;
} TuningParams_t;

void  BT_Init(void);
bool  BT_Poll(TuningParams_t *params);       /* 收到新参数返回 true */
void  BT_Send(const char *str);              /* 发送字符串 */
void  BT_SendParams(const TuningParams_t *p); /* 发送当前参数 */
bool  BT_IsConnected(void);                  /* 查询蓝牙连接状态 */

/* UART3 RX 中断: 收字节进缓冲 */
void UART_PB_INST_IRQHandler(void);

#endif
