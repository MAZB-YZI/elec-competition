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

#define RX_BUF_SIZE 64

static char     rx_buf[RX_BUF_SIZE];
static uint8_t  rx_idx;
static bool     rx_ready;

/* 连接状态追踪 */
static volatile uint32_t g_bt_last_rx_tick;   /* 上次收到数据的 tick 计数 */
static volatile bool     g_bt_connected;       /* 当前连接状态 */

static TuningParams_t g_params;   /* 本地拷贝, BT_Poll 时回填 */

/* ================================================================
 *  UART3 RX 中断: 收字符
 * ================================================================ */
void UART_PB_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_PB_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        g_bt_last_rx_tick = 0;  /* 有数据 = 已连接，清零超时计数 */
        g_bt_connected    = true;

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
    rx_idx   = 0;
    rx_ready = false;
    g_bt_last_rx_tick = 0;
    g_bt_connected    = false;

    /* SysConfig 已初始化 UART3, 只需要使能 RX 中断 */
    DL_UART_Main_enableInterrupt(UART_PB_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_PB_INST_INT_IRQN);

#ifdef BT_USE_STATE_PIN
    /* STATE 引脚: 输入模式，检测 HC-05 连接状态 */
    DL_GPIO_initDigitalInput(BT_STATE_PORT, BT_STATE_PIN);
#endif
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
 *  BT_IsConnected: 查询蓝牙连接状态
 *
 *  优先读 STATE 引脚, 否则靠超时判断
 *  ISR 中 g_bt_last_rx_tick 在收到数据时清零
 *  本函数由主循环周期性调用，tick_ms 为调用间隔（如 5ms）
 * ================================================================ */
bool BT_IsConnected(void)
{
#ifdef BT_USE_STATE_PIN
    /* 硬件 STATE 引脚: HC-05 连接时输出高电平 */
    return DL_GPIO_readPins(BT_STATE_PORT, BT_STATE_PIN) != 0;
#else
    /* 软件超时: 超过 BT_TIMEOUT_MS 无数据 → 断开 */
    return g_bt_connected;
#endif
}

/* ================================================================
 *  BT_Poll: 主循环调用, 有新命令返回 true
 *
 *  tick_ms: 调用间隔 (与 CTRL_TIMER 一致, 5ms)
 * ================================================================ */
bool BT_Poll(TuningParams_t *params)
{
    bool updated = false;

    /* 超时断连检测 (软件模式) */
#ifndef BT_USE_STATE_PIN
    g_bt_last_rx_tick++;
    if (g_bt_last_rx_tick > BT_TIMEOUT_MS / 5) {
        g_bt_connected = false;
    }
#endif

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
