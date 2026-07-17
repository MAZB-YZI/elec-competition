/**
 * bluetooth.h — HC-05 蓝牙串口调参 (UART3, PB2/TX, PB3/RX)
 */

#ifndef HCAR_BLUETOOTH_H
#define HCAR_BLUETOOTH_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 可调参数结构体（字段名避免与宏冲突） */
typedef struct {
    float   kp;             /* 循迹比例 */
    float   kd;             /* 微分 */
    int16_t base_pwm;       /* 循迹基准 PWM */
    int16_t heading_kp;     /* 航向比例 */
    int16_t speed_kp;       /* 速度环比例 */
} BT_Params_t;

void  BT_Init(void);
void  BT_SetTickPtr(volatile uint32_t *tick_ptr);  /* 传入 g_ms_ticks 指针 */
void  BT_SetParams(const BT_Params_t *init);        /* 同步初始参数 */
bool  BT_Poll(BT_Params_t *params);        /* 收到新参数返回 true */
void  BT_Send(const char *str);             /* 发送字符串 */
void  BT_Printf(const char *fmt, ...);      /* 格式化发送 */
void  BT_SendStatus(int16_t dist, int16_t spd_l, int16_t spd_r, float yaw);  /* 回传状态 */

/* UART3 RX 中断处理 */
void UART_BT_INST_IRQHandler(void);

#endif
