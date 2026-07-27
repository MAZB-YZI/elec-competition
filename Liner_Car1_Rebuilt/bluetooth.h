/**
 * bluetooth.h — HC-05 蓝牙串口调参
 */

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "ti_msp_dl_config.h"
#include <stdbool.h>

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

/* UART3 RX 中断: 收字节进环形缓冲 */
void UART_PB_INST_IRQHandler(void);

#endif
