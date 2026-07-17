/**
 * bluetooth.c — HC-05 蓝牙串口通用模块
 *
 * 提供 UART RX/TX 基础设施，命令解析由项目自定义
 * 通过 BT_SetCmdHandler 注册命令处理回调
 */

#include "bluetooth.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static char     rx_buf[BT_RX_BUF_SIZE];
static uint8_t  rx_idx = 0;
static bool     rx_ready = false;
static uint32_t rx_last_ms = 0;
static volatile uint32_t *bt_tick = NULL;

static BT_CmdHandler cmd_handler = NULL;
static void *cmd_ctx = NULL;

/* ================================================================
 *  UART RX 中断处理
 * ================================================================ */
void BT_UartIrqHandler(void)
{
    /* 项目需在实际 UART 中断向量中调用此函数 */
    /* 例如: void UART_BT_INST_IRQHandler(void) { BT_UartIrqHandler(); } */
}

/* 通用中断处理（由项目特定的中断向量调用） */
static void bt_rx_isr(void)
{
    if (rx_idx < BT_RX_BUF_SIZE - 1) {
        uint8_t ch = DL_UART_Main_receiveData(UART_BT_INST);
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
}

/* ================================================================
 *  BT_Init
 * ================================================================ */
void BT_Init(void)
{
    rx_idx = 0;
    rx_ready = false;
    rx_last_ms = 0;

    DL_UART_Main_enableInterrupt(UART_BT_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_BT_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BT_INST_INT_IRQN);
}

void BT_SetTickPtr(volatile uint32_t *tick_ptr)
{
    bt_tick = tick_ptr;
}

void BT_SetCmdHandler(BT_CmdHandler handler, void *ctx)
{
    cmd_handler = handler;
    cmd_ctx = ctx;
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

const char *BT_GetRxBuf(void)
{
    return rx_buf;
}

/* ================================================================
 *  BT_Poll
 * ================================================================ */
bool BT_Poll(void)
{
    bool updated = false;

    /* 超时处理：收到字符后 BT_TIMEOUT_MS 没换行也当命令处理 */
    if (!rx_ready && rx_idx > 0 && bt_tick && (*bt_tick - rx_last_ms > BT_TIMEOUT_MS)) {
        rx_buf[rx_idx] = '\0';
        rx_ready = true;
        rx_idx = 0;
    }

    if (rx_ready) {
        rx_ready = false;
        if (cmd_handler) {
            updated = cmd_handler(rx_buf, cmd_ctx);
        }
    }

    return updated;
}
