#include "bluetooth.h"
#include "route_fsm.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_BUF_SIZE 128U
#define RX_TIMEOUT_MS 200U

static char rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_idx;
static volatile bool rx_ready;
static volatile uint32_t rx_last_ms;
static volatile uint32_t *bt_tick;

static TuningParams_t g_params;
static char last_cmd[RX_BUF_SIZE];
static uint32_t last_cmd_ms;

static void bt_send_char(uint8_t ch)
{
    uint32_t guard = 10000U;

    while ((guard-- > 0U) && !DL_UART_Main_transmitDataCheck(UART_PB_INST, ch)) {
    }
}

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
    /* 1. 先清软件状态 */
    rx_idx = 0U;
    rx_ready = false;
    rx_last_ms = 0U;
    bt_tick = NULL;
    memset(&g_params, 0, sizeof(g_params));
    memset(last_cmd, 0, sizeof(last_cmd));
    last_cmd_ms = 0U;

    /* 2. 清硬件接收寄存器（最多读16次，防止无限循环） */
    for (uint8_t i = 0; i < 16U; i++) {
        if (DL_UART_getRawInterruptStatus(UART_PB_INST,
                DL_UART_INTERRUPT_RX) == 0U) break;
        (void)DL_UART_receiveData(UART_PB_INST);
    }
    DL_UART_clearInterruptStatus(UART_PB_INST, DL_UART_INTERRUPT_RX);

    /* 3. 最后才开启中断 */
    NVIC_ClearPendingIRQ(UART_PB_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_PB_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_PB_INST, DL_UART_MAIN_INTERRUPT_RX);
}

void BT_SetTickPtr(volatile uint32_t *tick_ptr)
{
    bt_tick = tick_ptr;
}

void BT_Send(const char *str)
{
    while (*str != '\0') {
        bt_send_char((uint8_t)*str++);
    }
}

static char bt_tx_buf[256];  /* 文件级静态缓冲区，不占栈空间 */

void BT_Printf(const char *fmt, ...)
{
    va_list ap;
    int len;

    va_start(ap, fmt);
    len = vsnprintf(bt_tx_buf, sizeof(bt_tx_buf), fmt, ap);
    va_end(ap);

    if (len < 0) {
        return;
    }
    if (len >= (int)sizeof(bt_tx_buf)) {
        len = (int)sizeof(bt_tx_buf) - 1;
    }

    for (int i = 0; i < len; i++) {
        bt_send_char((uint8_t)bt_tx_buf[i]);
    }
}

void BT_SendParams(const TuningParams_t *p)
{
    BT_Printf("M%d %s T%.1fs KP%.2f KD%.2f B%d L%d TR%d F%d-%d C%d Bk%d To%.1f La%.1f\r\n",
              (int)Route_GetMode(),
              Route_GetStateName(),
              (float)Route_GetElapsedMs() / 1000.0f,
              Route_GetKp(),
              Route_GetKd(),
              Route_GetBasePwm(),
              Route_GetOutputLim(),
              Route_GetTrim(),
              (int)Route_GetFinishMinDist(),
              (int)Route_GetFinishMaxDist(),
              (int)Route_GetFinishConfirmTicks(),
              (int)Route_GetBrakeDurationMs(),
              (float)Route_GetTimeoutMs() / 1000.0f,
              (float)Route_GetLapTargetMs() / 1000.0f);
    /* 运行结束后输出结果 */
    if (Route_IsFinished()) {
        BT_Printf("RESULT=%s T=%.2f D=%.1f PD=%.1f PB=%d PT=%.2f RAW=0x%02X\r\n",
                  Route_GetFinishReasonStr(),
                  (float)Route_GetElapsedMs() / 1000.0f,
                  Route_GetDistanceCm(),
                  Route_GetPeakDist(),
                  Route_GetPeakBc(),
                  (float)Route_GetPeakMs() / 1000.0f,
                  Route_GetPeakRaw());
    }
}

/* 去掉首尾空格和 \r \n */
static void trim_cmd(char *cmd)
{
    /* 去尾部空白和换行 */
    int len = (int)strlen(cmd);
    while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\r' ||
                       cmd[len - 1] == '\n' || cmd[len - 1] == '\t')) {
        cmd[--len] = '\0';
    }
    /* 去首部空白 */
    char *p = cmd;
    while (*p == ' ' || *p == '\t') p++;
    if (p != cmd) {
        memmove(cmd, p, strlen(p) + 1);
    }
}

static bool parse_cmd(const char *cmd, TuningParams_t *p)
{
    float val;
    int ival;

    if (strcmp(cmd, "hello") == 0) {
        BT_Send("receive:hello\r\n");
        return false;
    }
    if (sscanf(cmd, "KP %f", &val) == 1) {
        p->KP = val;
        Route_SetKp(val);
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
        Route_SetBasePwm((int16_t)ival);
        BT_Printf("OK BASE=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "KD %f", &val) == 1) {
        Route_SetKd(val);
        BT_Printf("OK KD=%.2f\r\n", val);
        return false;
    }
    if (sscanf(cmd, "LIM %d", &ival) == 1) {
        p->OUTPUT_LIM = (int16_t)ival;
        Route_SetOutputLim((int16_t)ival);
        BT_Printf("OK LIM=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "FMIN %f", &val) == 1) {
        Route_SetFinishMinDist(val);
        BT_Printf("OK FMIN=%.0f\r\n", val);
        return false;
    }
    if (sscanf(cmd, "FMAX %f", &val) == 1) {
        Route_SetFinishMaxDist(val);
        BT_Printf("OK FMAX=%.0f\r\n", val);
        return false;
    }
    if (sscanf(cmd, "FCNT %d", &ival) == 1) {
        Route_SetFinishConfirmTicks((uint8_t)ival);
        BT_Printf("OK FCNT=%d\r\n", ival);
        return false;
    }
    if (sscanf(cmd, "BRAKE %d", &ival) == 1) {
        Route_SetBrakeDurationMs((uint16_t)ival);
        BT_Printf("OK BRAKE=%d\r\n", ival);
        return false;
    }
    if (sscanf(cmd, "TOUT %f", &val) == 1) {
        Route_SetTimeoutMs((uint32_t)(val * 1000.0f));
        BT_Printf("OK TOUT=%.1fs\r\n", val);
        return false;
    }
    if (sscanf(cmd, "LAP %f", &val) == 1) {
        Route_SetLapTargetMs((uint32_t)(val * 1000.0f));
        BT_Printf("OK LAP=%.1fs\r\n", val);
        return false;
    }
    if (sscanf(cmd, "TRIM %d", &ival) == 1) {
        Route_SetTrim((int16_t)ival);
        BT_Printf("OK TRIM=%d\r\n", ival);
        return false;
    }
    if (sscanf(cmd, "TURN %d", &ival) == 1) {
        p->TURN_SPEED = (int16_t)ival;
        BT_Printf("OK TURN=%d\r\n", ival);
        return true;
    }
    if (sscanf(cmd, "MODE %d", &ival) == 1) {
        Route_SetMode((RouteMode_t)ival);
        BT_Printf("MODE %d OK\r\n", ival);
        return false;
    }
    if (strcmp(cmd, "START") == 0) {
        g_route_start_request = true;
        BT_Send("START OK\r\n");
        return false;
    }
    if (strcmp(cmd, "STOP") == 0) {
        g_route_stop_request = true;
        BT_Send("STOP OK\r\n");
        return false;
    }
    if (strcmp(cmd, "SHOW") == 0) {
        BT_SendParams(p);
        return false;
    }

    BT_Printf("UNKNOWN [%s]\r\n", cmd);
    return false;
}

bool BT_Poll(TuningParams_t *params)
{
    bool updated = false;

    /* 超时成帧: 200ms 无换行则当作完整命令（兼容不发换行的蓝牙软件） */
    if ((rx_ready == false) && (rx_idx > 0U) && (bt_tick != NULL) &&
        ((*bt_tick - rx_last_ms) > RX_TIMEOUT_MS)) {
        rx_buf[rx_idx] = '\0';
        rx_ready = true;
        rx_idx = 0U;
    }

    if (rx_ready) {
        rx_ready = false;

        /* 去首尾空格和换行 */
        trim_cmd(rx_buf);

        /* 空命令不处理 */
        if (rx_buf[0] == '\0') {
            return false;
        }

        /* 防止蓝牙回显导致重复处理 */
        if (bt_tick != NULL && strcmp(rx_buf, last_cmd) == 0 &&
            (*bt_tick - last_cmd_ms) < 100U) {
            return false;
        }
        strncpy(last_cmd, rx_buf, RX_BUF_SIZE - 1);
        last_cmd[RX_BUF_SIZE - 1] = '\0';
        if (bt_tick != NULL) last_cmd_ms = *bt_tick;
        g_params = *params;
        updated = parse_cmd(rx_buf, &g_params);
        if (updated) {
            *params = g_params;
        }
    }

    return updated;
}
