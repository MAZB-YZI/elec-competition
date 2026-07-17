/**
 * bluetooth.c — HC-05 蓝牙串口调参 (UART3, PB2/TX, PB3/RX)
 *
 * 命令格式:
 *   LKP 2.0     设置循迹 KP
 *   LKD 0.1     设置循迹 KD
 *   BASE 600    设置循迹基准 PWM
 *   HKP 1.0     设置航向 KP
 *   SKP 500     设置速度环 KP
 *   SHOW        回传当前参数
 *   hello       通信测试（返回 receive:hello）
 */

#include "bluetooth.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#define RX_BUF_SIZE 128

static char     rx_buf[RX_BUF_SIZE];
static uint8_t  rx_idx = 0;
static bool     rx_ready = false;
static uint32_t rx_last_ms = 0;
static volatile uint32_t *bt_tick = NULL;

static BT_Params_t g_params;

/* ================================================================
 *  UART3 RX 中断
 * ================================================================ */
void UART_BT_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BT_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        if (rx_idx < RX_BUF_SIZE - 1) {
            uint8_t ch = DL_UART_Main_receiveDataBlocking(UART_BT_INST);
            if (bt_tick) rx_last_ms = *bt_tick;
            if (ch == '\n' || ch == '\r') {
                if (rx_idx > 0) {
                    rx_buf[rx_idx] = '\0';
                    rx_ready = true;
                }
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
 *  BT_Init
 * ================================================================ */
void BT_Init(void)
{
    rx_idx  = 0;
    rx_ready = false;
    rx_last_ms = 0;
    memset(&g_params, 0, sizeof(g_params));

    DL_UART_Main_enableInterrupt(UART_BT_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_BT_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BT_INST_INT_IRQN);
}

void BT_SetTickPtr(volatile uint32_t *tick_ptr)
{
    bt_tick = tick_ptr;
}

void BT_SetParams(const BT_Params_t *init)
{
    g_params = *init;
}

/* ================================================================
 *  BT_Send / BT_Printf
 * ================================================================ */
void BT_Send(const char *str)
{
    while (*str) {
        DL_UART_Main_transmitDataBlocking(UART_BT_INST, (uint8_t)*str++);
    }
}

void BT_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    for (int i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_BT_INST, (uint8_t)buf[i]);
    }
}

/* ================================================================
 *  BT_SendParams
 * ================================================================ */
static void BT_SendParams(const BT_Params_t *p)
{
    BT_Printf("LKP=%.2f LKD=%.2f BASE=%d HKP=%d SKP=%d\r\n",
              p->kp, p->kd, p->base_pwm, p->heading_kp, p->speed_kp);
}

/* ================================================================
 *  命令解析
 * ================================================================ */
static bool parse_cmd(const char *cmd, BT_Params_t *p)
{
    float  fval;
    int    ival;

    if (strstr(cmd, "hello")) {
        BT_Send("receive:hello\r\n");
        return false;
    }

    if (sscanf(cmd, "LKP %f", &fval) == 1) {
        p->kp = fval;
        BT_Printf("OK LKP=%.2f\r\n", fval);
        return true;
    }
    if (sscanf(cmd, "LKD %f", &fval) == 1) {
        p->kd = fval;
        BT_Printf("OK LKD=%.2f\r\n", fval);
        return true;
    }
    if (sscanf(cmd, "BASE %d", &ival) == 1) {
        p->base_pwm = (int16_t)ival;
        BT_Printf("OK BASE=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "HKP %d", &ival) == 1) {
        p->heading_kp = (int16_t)ival;
        BT_Printf("OK HKP=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "SKP %d", &ival) == 1) {
        p->speed_kp = (int16_t)ival;
        BT_Printf("OK SKP=%d\r\n", ival);
        return true;
    }
    if (strstr(cmd, "SHOW")) {
        BT_SendParams(p);
        return false;
    }

    BT_Send("UNKNOWN CMD\r\n");
    return false;
}

/* ================================================================
 *  BT_Poll
 * ================================================================ */
bool BT_Poll(BT_Params_t *params)
{
    bool updated = false;

    /* 超时处理：收到字符后200ms没换行也当命令处理 */
    if (!rx_ready && rx_idx > 0 && bt_tick && (*bt_tick - rx_last_ms > 200U)) {
        rx_buf[rx_idx] = '\0';
        rx_ready = true;
        rx_idx = 0;
    }

    if (rx_ready) {
        rx_ready = false;
        updated = parse_cmd(rx_buf, &g_params);
        if (updated) {
            *params = g_params;
        }
    }

    return updated;
}
