#include "bluetooth.h"
#include "route_fsm.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_BUF_SIZE 128U
#define RX_TIMEOUT_MS 200U

static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_idx;
static bool rx_ready;
static uint32_t rx_last_ms;
static volatile uint32_t *bt_tick;

static TuningParams_t g_params;

void UART_PB_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_PB_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        if (rx_idx < RX_BUF_SIZE - 1U) {
            uint8_t ch = DL_UART_Main_receiveDataBlocking(UART_PB_INST);
            if (bt_tick != NULL) {
                rx_last_ms = *bt_tick;
            }
            if ((ch == '\n') || (ch == '\r')) {
                if (rx_idx > 0U) {
                    rx_buf[rx_idx] = '\0';
                    rx_ready = true;
                }
                rx_idx = 0U;
            } else {
                rx_buf[rx_idx++] = (char)ch;
            }
        } else {
            rx_idx = 0U;
        }
        break;

    default:
        break;
    }
}

void BT_Init(void)
{
    rx_idx = 0U;
    rx_ready = false;
    rx_last_ms = 0U;
    bt_tick = NULL;
    memset(&g_params, 0, sizeof(g_params));

    DL_UART_Main_enableInterrupt(UART_PB_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_PB_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_PB_INST_INT_IRQN);
}

void BT_SetTickPtr(volatile uint32_t *tick_ptr)
{
    bt_tick = tick_ptr;
}

void BT_Send(const char *str)
{
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_PB_INST, (uint8_t)*str++);
    }
}

void BT_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    int len;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len < 0) {
        return;
    }
    if (len > (int)sizeof(buf)) {
        len = (int)sizeof(buf);
    }

    for (int i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_PB_INST, (uint8_t)buf[i]);
    }
}

void BT_SendParams(const TuningParams_t *p)
{
    BT_Printf("KP=%.2f KI=%.2f BASE=%d TURN=%d LIM=%d ROUTE=%s\r\n",
              p->KP, p->KI, p->BASE_PWM, p->TURN_SPEED, p->OUTPUT_LIM,
              Route_GetStateName());
}

static bool parse_cmd(const char *cmd, TuningParams_t *p)
{
    float val;
    int ival;

    if (strstr(cmd, "hello") != NULL) {
        BT_Send("receive:hello\r\n");
        return false;
    }
    if (sscanf(cmd, "KP %f", &val) == 1) {
        p->KP = val;
        BT_Printf("OK KP=%.2f\r\n", val);
        return true;
    }
    if (sscanf(cmd, "KI %f", &val) == 1) {
        p->KI = val;
        BT_Printf("OK KI=%.2f\r\n", val);
        return true;
    }
    if (sscanf(cmd, "BASE %d", &ival) == 1) {
        p->BASE_PWM = (int16_t)ival;
        Route_SetStraightBase((int16_t)ival);
        BT_Printf("OK BASE=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "HKP %f", &val) == 1) {
        Route_SetHeadingKp(val);
        BT_Printf("OK HKP=%.2f\r\n", val);
        return false;
    }
    if (sscanf(cmd, "TRIM %d", &ival) == 1) {
        Route_SetStraightTrim((int16_t)ival);
        BT_Printf("OK TRIM=%d\r\n", ival);
        return false;
    }
    if (sscanf(cmd, "HSIGN %d", &ival) == 1) {
        Route_SetHeadingSign((int8_t)ival);
        BT_Printf("OK HSIGN=%d\r\n", (ival < 0) ? -1 : 1);
        return false;
    }
    if (sscanf(cmd, "ARC %d", &ival) == 1) {
        Route_SetArcBase((int16_t)ival);
        BT_Printf("OK ARC=%d\r\n", ival);
        return false;
    }
    if (sscanf(cmd, "LKP %f", &val) == 1) {
        Route_SetLineKp(val);
        BT_Printf("OK LKP=%.2f\r\n", val);
        return false;
    }
    if (sscanf(cmd, "LKD %f", &val) == 1) {
        Route_SetLineKd(val);
        BT_Printf("OK LKD=%.2f\r\n", val);
        return false;
    }
    if (sscanf(cmd, "ADIST %f", &val) == 1) {
        Route_SetArcDistCm(val);
        BT_Printf("OK ADIST=%.1f\r\n", val);
        return false;
    }
    if (sscanf(cmd, "TURN %d", &ival) == 1) {
        p->TURN_SPEED = (int16_t)ival;
        BT_Printf("OK TURN=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "LIM %d", &ival) == 1) {
        p->OUTPUT_LIM = (int16_t)ival;
        BT_Printf("OK LIM=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "MODE %d", &ival) == 1) {
        Route_SetMode((RouteMode_t)ival);
        BT_Send("MODE OK\r\n");
        return false;
    }
    if (strstr(cmd, "START") != NULL) {
        Route_Start();
        BT_Send("START OK\r\n");
        return false;
    }
    if (strstr(cmd, "STOP") != NULL) {
        Route_Stop();
        BT_Send("STOP OK\r\n");
        return false;
    }
    if (strstr(cmd, "SHOW") != NULL) {
        BT_SendParams(p);
        return false;
    }

    BT_Send("UNKNOWN CMD\r\n");
    return false;
}

bool BT_Poll(TuningParams_t *params)
{
    bool updated = false;

    if ((rx_ready == false) && (rx_idx > 0U) && (bt_tick != NULL) &&
        ((*bt_tick - rx_last_ms) > RX_TIMEOUT_MS)) {
        rx_buf[rx_idx] = '\0';
        rx_ready = true;
        rx_idx = 0U;
    }

    if (rx_ready) {
        rx_ready = false;
        g_params = *params;
        updated = parse_cmd(rx_buf, &g_params);
        if (updated) {
            *params = g_params;
        }
    }

    return updated;
}
