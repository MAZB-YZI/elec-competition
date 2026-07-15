/**
 * bluetooth.c — HC-05 蓝牙串口调参 (UART3, PB2/TX, PB3/RX)
 *
 * 命令格式:
 *   KP 3.0      设置 KP
 *   KI 0.2      设置 KI
 *   BASE 1500   设置基础速度
 *   TURN 2500   设置直角转向速度
 *   LIM 1200    设置输出限幅
 *   SHOW        回传当前参数
 */

#include "bluetooth.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define RX_BUF_SIZE 64

static char     rx_buf[RX_BUF_SIZE];
static uint8_t  rx_idx;
static bool     rx_ready;

static TuningParams_t g_params;   /* 本地拷贝, BT_Poll 时回填 */

/* ================================================================
 *  UART3 RX 中断: 收字符
 * ================================================================ */
void UART_PB_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_PB_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        if (rx_idx < RX_BUF_SIZE - 1) {
            uint8_t ch = DL_UART_Main_receiveDataBlocking(UART_PB_INST);
            if (ch == '\n' || ch == '\r') {
                rx_buf[rx_idx] = '\0';
                if (rx_idx > 0) rx_ready = true;
                rx_idx = 0;
            } else {
                rx_buf[rx_idx++] = (char)ch;
            }
        }
        break;

    default:
        break;
    }
}

/* ================================================================
 *  BT_Init: 注册 UART3 中断回调
 * ================================================================ */
void BT_Init(void)
{
    rx_idx  = 0;
    rx_ready = false;

    /* SysConfig 已初始化 UART3, 只需要使能 RX 中断 */
    DL_UART_Main_enableInterrupt(UART_PB_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_PB_INST_INT_IRQN);
}

/* ================================================================
 *  BT_Send: 发送字符串
 * ================================================================ */
void BT_Send(const char *str)
{
    while (*str) {
        DL_UART_Main_transmitDataBlocking(UART_PB_INST, (uint8_t)*str++);
    }
}

/* ================================================================
 *  BT_SendParams: 回传当前参数
 * ================================================================ */
void BT_SendParams(const TuningParams_t *p)
{
    char buf[64];
    int  len = snprintf(buf, sizeof(buf),
        "KP=%.2f KI=%.2f BASE=%d TURN=%d LIM=%d\r\n",
        p->KP, p->KI, p->BASE_PWM, p->TURN_SPEED, p->OUTPUT_LIM);
    for (int i = 0; i < len; i++)
        DL_UART_Main_transmitDataBlocking(UART_PB_INST, (uint8_t)buf[i]);
}

/* ================================================================
 *  命令解析
 * ================================================================ */
static bool parse_cmd(const char *cmd, TuningParams_t *p)
{
    char  key[8];
    float val;
    int   ival;

    /* KP x.x */
    if (sscanf(cmd, "KP %f", &val) == 1) {
        p->KP = val; return true;
    }
    /* KI x.x */
    if (sscanf(cmd, "KI %f", &val) == 1) {
        p->KI = val; return true;
    }
    /* BASE xxxx */
    if (sscanf(cmd, "BASE %d", &ival) == 1) {
        p->BASE_PWM = (int16_t)ival; return true;
    }
    /* TURN xxxx */
    if (sscanf(cmd, "TURN %d", &ival) == 1) {
        p->TURN_SPEED = (int16_t)ival; return true;
    }
    /* LIM xxxx */
    if (sscanf(cmd, "LIM %d", &ival) == 1) {
        p->OUTPUT_LIM = (int16_t)ival; return true;
    }
    /* SHOW */
    if (strstr(cmd, "SHOW")) {
        BT_SendParams(p);
    }
    return false;
}

/* ================================================================
 *  BT_Poll: 主循环调用, 有新命令返回 true
 * ================================================================ */
bool BT_Poll(TuningParams_t *params)
{
    bool updated = false;

    if (rx_ready) {
        rx_ready = false;
        updated = parse_cmd(rx_buf, &g_params);

        /* 每次收到命令都回传确认 */
        BT_SendParams(&g_params);

        if (updated) {
            *params = g_params;
        }
    }

    return updated;
}
