/**
 * bluetooth.c — 蓝牙串口命令解析
 *
 * 支持巡线调参、停车参数、控制命令、HELP、SHOW、TEL 遥测
 */

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

static char last_cmd[RX_BUF_SIZE];
static uint32_t last_cmd_ms;

/* 遥测 */
static volatile uint16_t g_tel_period_ms = 0;

static void bt_send_char(uint8_t ch)
{
    uint32_t guard = 10000U;
    while ((guard-- > 0U) && !DL_UART_Main_transmitDataCheck(UART_PB_INST, ch)) {}
}

void UART_PB_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_PB_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        if (rx_idx < RX_BUF_SIZE - 1U) {
            uint8_t ch = DL_UART_Main_receiveDataBlocking(UART_PB_INST);
            if (bt_tick != NULL) rx_last_ms = *bt_tick;
            if ((ch == '\n') || (ch == '\r')) {
                if (rx_idx > 0U) { rx_buf[rx_idx] = '\0'; rx_ready = true; }
                rx_idx = 0U;
            } else {
                rx_buf[rx_idx++] = (char)ch;
            }
        } else {
            rx_idx = 0U;
        }
        break;
    default: break;
    }
}

void BT_Init(void)
{
    rx_idx = 0U; rx_ready = false; rx_last_ms = 0U; bt_tick = NULL;
    memset(last_cmd, 0, sizeof(last_cmd)); last_cmd_ms = 0U;
    g_tel_period_ms = 0;

    for (uint8_t i = 0; i < 16U; i++) {
        if (DL_UART_getRawInterruptStatus(UART_PB_INST, DL_UART_INTERRUPT_RX) == 0U) break;
        (void)DL_UART_receiveData(UART_PB_INST);
    }
    DL_UART_clearInterruptStatus(UART_PB_INST, DL_UART_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_PB_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_PB_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_PB_INST, DL_UART_MAIN_INTERRUPT_RX);
}

void BT_SetTickPtr(volatile uint32_t *tick_ptr) { bt_tick = tick_ptr; }

void BT_Send(const char *str)
{
    while (*str != '\0') bt_send_char((uint8_t)*str++);
}

static char bt_tx_buf[256];

void BT_Printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(bt_tx_buf, sizeof(bt_tx_buf), fmt, ap);
    va_end(ap);
    if (len < 0) return;
    if (len > (int)sizeof(bt_tx_buf)) len = (int)sizeof(bt_tx_buf);
    for (int i = 0; i < len; i++) bt_send_char((uint8_t)bt_tx_buf[i]);
}

/* ── SHOW: 分组打印 ── */
void BT_SendParams(void)
{
    BT_Printf("STATUS M=%d ST=%s T=%.2f D=%.1f\r\n",
              (int)Route_GetMode(), Route_GetStateName(),
              (float)Route_GetElapsedMs() / 1000.0f, Route_GetDistanceCm());

    BT_Printf("LINE KP=%.2f KD=%.2f BASE=%d LIM=%d TR=%d DZ=%d SL=%d\r\n",
              Route_GetKp(), Route_GetKd(), Route_GetBasePwm(),
              Route_GetOutputLim(), Route_GetTrim(),
              Route_GetDeadZone(), Route_GetSlewStep());

    BT_Printf("STOP F=%.0f-%.0f BTH=%u C=%u BK=%u LOST=%u SC=%.0f\r\n",
              Route_GetFinishMinDist(), Route_GetFinishMaxDist(),
              Route_GetBlackMin(), Route_GetFinishConfirmTicks(),
              Route_GetBrakeDurationMs(), Route_GetLostMs(),
              Route_GetStartClearCm());

    BT_Printf("TIME LAP=%.2f TW=%.2f SR=%.2f SA=%.2f FBA=%.2f TO=%.2f\r\n",
              Route_GetLapTargetMs() / 1000.0f,
              Route_GetTimeWindowMs() / 1000.0f,
              Route_GetSlowRatio(),
              Route_GetSlowAheadMs() / 1000.0f,
              Route_GetFallbackAheadMs() / 1000.0f,
              Route_GetTimeoutMs() / 1000.0f);

    BT_Printf("Q4  PWM=%d RAMP=%u ARM=%.0f B=%.0f HKP=%.2f HLIM=%d AB=%.2f\r\n",
              Route_GetQ4Pwm(), Route_GetQ4RampMs(),
              Route_GetQ4ArmCm(), Route_GetQ4BCm(),
              Route_GetQ4Hkp(), Route_GetQ4Hlim(),
              Route_GetQ4AbTimeMs() / 1000.0f);

    BT_Printf("Q5  PWM=%d RAMP=%u OFF=%.1f POST=%.0f STOP=%u TO=%.1f TMIN=%.1f TMAX=%.1f PASS=%d LAP=%.2f\r\n",
              Route_GetQ5Pwm(), Route_GetQ5RampMs(), Route_GetQ5OffsetCm(),
              Route_GetQ5PostCm(), Route_GetQ5StopMs(),
              Route_GetQ5TimeoutMs() / 1000.0f,
              Route_GetQ5DetectMinMs() / 1000.0f,
              Route_GetQ5DetectMaxMs() / 1000.0f,
              Route_GetQ5PassedA() ? 1 : 0,
              Route_GetQ5LapTimeMs() / 1000.0f);

    if (Route_IsFinished()) {
        BT_Printf("RESULT=%s T=%.2f D=%.1f\r\n",
                  Route_GetFinishReasonStr(),
                  (float)Route_GetElapsedMs() / 1000.0f,
                  Route_GetDistanceCm());
        BT_Printf("PB=%u PT=%.2f PD=%.1f RAW=0x%02X\r\n",
                  Route_GetPeakBc(), (float)Route_GetPeakMs() / 1000.0f,
                  Route_GetPeakDist(), Route_GetPeakRaw());
    }

    /* 标定模式: 显示编码器详情 */
    if (Route_GetMode() == ROUTE_MODE_CALIBRATE) {
        float dist = Route_GetDistanceCm();
        BT_Printf("CAL D=%.1f T=%.1f\r\n", dist,
                  (float)Route_GetElapsedMs() / 1000.0f);
    }
}

/* ── HELP ── */
static void BT_SendHelp(void)
{
    BT_Send("LINE: KP KD BASE LIM TRIM DZ SLEW\r\n");
    BT_Send("STOP: FMIN FMAX BTH FCNT BRAKE LOST SC\r\n");
    BT_Send("TIME: LAP TW SLOWR SLOWA FBA TOUT\r\n");
    BT_Send("Q4  : Q4PWM Q4RAMP Q4ARM Q4B Q4HKP Q4HLIM\r\n");
    BT_Send("Q5  : Q5PWM Q5RAMP Q5OFFSET Q5POST Q5STOP Q5TOUT Q5TMIN Q5TMAX\r\n");
    BT_Send("CTRL: MODE START STOP SHOW HELP ZERO\r\n");
    BT_Send("TEL : TEL 100 / TEL 0\r\n");
}

/* ── 去首尾空格 ── */
static void trim_cmd(char *cmd)
{
    int len = (int)strlen(cmd);
    while (len > 0 && (cmd[len-1]==' '||cmd[len-1]=='\r'||cmd[len-1]=='\n'||cmd[len-1]=='\t'))
        cmd[--len] = '\0';
    char *p = cmd;
    while (*p == ' ' || *p == '\t') p++;
    if (p != cmd) memmove(cmd, p, strlen(p) + 1);
}

/* ── 命令解析 ── */
static void parse_cmd(const char *cmd)
{
    float val; int ival;

    /* 控制 */
    if (strcmp(cmd, "START") == 0) {
        if (Route_IsActive()) { BT_Send("ERR ALREADY RUNNING\r\n"); return; }
        g_route_start_request = true; BT_Send("START OK\r\n"); return;
    }
    if (strcmp(cmd, "STOP") == 0) {
        g_route_stop_request = true; BT_Send("STOP OK\r\n"); return;
    }
    if (strcmp(cmd, "SHOW") == 0) { BT_SendParams(); return; }
    if (strcmp(cmd, "ZERO") == 0) { Encoder_ResetDistance(); BT_Send("ZERO OK\r\n"); return; }
    if (strcmp(cmd, "HELP") == 0) { BT_SendHelp(); return; }
    if (strcmp(cmd, "hello") == 0) { BT_Send("receive:hello\r\n"); return; }

    /* MODE */
    if (sscanf(cmd, "MODE %d", &ival) == 1) {
        Route_SetMode((RouteMode_t)ival);
        BT_Printf("OK MODE=%d\r\n", (int)Route_GetMode()); return;
    }

    /* 巡线参数 */
    if (sscanf(cmd, "KP %f", &val) == 1) {
        Route_SetKp(val); BT_Printf("OK KP=%.2f\r\n", Route_GetKp()); return;
    }
    if (sscanf(cmd, "KD %f", &val) == 1) {
        Route_SetKd(val); BT_Printf("OK KD=%.2f\r\n", Route_GetKd()); return;
    }
    if (sscanf(cmd, "BASE %d", &ival) == 1) {
        Route_SetBasePwm((int16_t)ival); BT_Printf("OK BASE=%d\r\n", Route_GetBasePwm()); return;
    }
    if (sscanf(cmd, "LIM %d", &ival) == 1) {
        Route_SetOutputLim((int16_t)ival); BT_Printf("OK LIM=%d\r\n", Route_GetOutputLim()); return;
    }
    if (sscanf(cmd, "TRIM %d", &ival) == 1) {
        Route_SetTrim((int16_t)ival); BT_Printf("OK TRIM=%d\r\n", Route_GetTrim()); return;
    }
    if (sscanf(cmd, "DZ %d", &ival) == 1) {
        Route_SetDeadZone((int16_t)ival); BT_Printf("OK DZ=%d\r\n", Route_GetDeadZone()); return;
    }
    if (sscanf(cmd, "SLEW %d", &ival) == 1) {
        Route_SetSlewStep((int16_t)ival); BT_Printf("OK SLEW=%d\r\n", Route_GetSlewStep()); return;
    }

    /* 停车参数 */
    if (sscanf(cmd, "FMIN %f", &val) == 1) {
        Route_SetFinishMinDist(val); BT_Printf("OK FMIN=%.0f\r\n", Route_GetFinishMinDist()); return;
    }
    if (sscanf(cmd, "FMAX %f", &val) == 1) {
        Route_SetFinishMaxDist(val); BT_Printf("OK FMAX=%.0f\r\n", Route_GetFinishMaxDist()); return;
    }
    if (sscanf(cmd, "BTH %d", &ival) == 1) {
        Route_SetBlackMin((uint8_t)ival); BT_Printf("OK BTH=%u\r\n", Route_GetBlackMin()); return;
    }
    if (sscanf(cmd, "FCNT %d", &ival) == 1) {
        Route_SetFinishConfirmTicks((uint8_t)ival); BT_Printf("OK FCNT=%u\r\n", Route_GetFinishConfirmTicks()); return;
    }
    if (sscanf(cmd, "BRAKE %d", &ival) == 1) {
        Route_SetBrakeDurationMs((uint16_t)ival); BT_Printf("OK BRAKE=%u\r\n", Route_GetBrakeDurationMs()); return;
    }
    if (sscanf(cmd, "LOST %d", &ival) == 1) {
        Route_SetLostMs((uint16_t)ival); BT_Printf("OK LOST=%u\r\n", Route_GetLostMs()); return;
    }
    if (sscanf(cmd, "SC %f", &val) == 1) {
        Route_SetStartClearCm(val); BT_Printf("OK SC=%.0f\r\n", Route_GetStartClearCm()); return;
    }

    /* 时间参数 */
    if (sscanf(cmd, "LAP %f", &val) == 1) {
        Route_SetLapTargetMs((uint32_t)(val * 1000.0f));
        BT_Printf("OK LAP=%.2f\r\n", Route_GetLapTargetMs() / 1000.0f); return;
    }
    if (sscanf(cmd, "TW %f", &val) == 1) {
        Route_SetTimeWindowMs((uint32_t)(val * 1000.0f));
        BT_Printf("OK TW=%.2f\r\n", Route_GetTimeWindowMs() / 1000.0f); return;
    }
    if (sscanf(cmd, "SLOWR %f", &val) == 1) {
        Route_SetSlowRatio(val); BT_Printf("OK SLOWR=%.2f\r\n", Route_GetSlowRatio()); return;
    }
    if (sscanf(cmd, "SLOWA %f", &val) == 1) {
        Route_SetSlowAheadMs((uint32_t)(val * 1000.0f));
        BT_Printf("OK SLOWA=%.2f\r\n", Route_GetSlowAheadMs() / 1000.0f); return;
    }
    if (sscanf(cmd, "FBA %f", &val) == 1) {
        Route_SetFallbackAheadMs((uint32_t)(val * 1000.0f));
        BT_Printf("OK FBA=%.2f\r\n", Route_GetFallbackAheadMs() / 1000.0f); return;
    }
    if (sscanf(cmd, "TOUT %f", &val) == 1) {
        Route_SetTimeoutMs((uint32_t)(val * 1000.0f));
        BT_Printf("OK TOUT=%.2f\r\n", Route_GetTimeoutMs() / 1000.0f); return;
    }

    /* Q4 参数 */
    if (sscanf(cmd, "Q4PWM %d", &ival) == 1) {
        Route_SetQ4Pwm((int16_t)ival); BT_Printf("OK Q4PWM=%d\r\n", Route_GetQ4Pwm()); return;
    }
    if (sscanf(cmd, "Q4RAMP %d", &ival) == 1) {
        Route_SetQ4RampMs((uint16_t)ival); BT_Printf("OK Q4RAMP=%u\r\n", Route_GetQ4RampMs()); return;
    }
    if (sscanf(cmd, "Q4ARM %f", &val) == 1) {
        Route_SetQ4ArmCm(val); BT_Printf("OK Q4ARM=%.0f\r\n", Route_GetQ4ArmCm()); return;
    }
    if (sscanf(cmd, "Q4B %f", &val) == 1) {
        Route_SetQ4BCm(val); BT_Printf("OK Q4B=%.0f\r\n", Route_GetQ4BCm()); return;
    }
    if (sscanf(cmd, "Q4HKP %f", &val) == 1) {
        Route_SetQ4Hkp(val); BT_Printf("OK Q4HKP=%.2f\r\n", Route_GetQ4Hkp()); return;
    }
    if (sscanf(cmd, "Q4HLIM %d", &ival) == 1) {
        Route_SetQ4Hlim((int16_t)ival); BT_Printf("OK Q4HLIM=%d\r\n", Route_GetQ4Hlim()); return;
    }

    /* Q5 参数 */
    if (sscanf(cmd, "Q5PWM %d", &ival) == 1) {
        Route_SetQ5Pwm((int16_t)ival); BT_Printf("OK Q5PWM=%d\r\n", Route_GetQ5Pwm()); return;
    }
    if (sscanf(cmd, "Q5RAMP %d", &ival) == 1) {
        Route_SetQ5RampMs((uint16_t)ival); BT_Printf("OK Q5RAMP=%u\r\n", Route_GetQ5RampMs()); return;
    }
    if (sscanf(cmd, "Q5OFFSET %f", &val) == 1) {
        Route_SetQ5OffsetCm(val); BT_Printf("OK Q5OFFSET=%.1f\r\n", Route_GetQ5OffsetCm()); return;
    }
    if (sscanf(cmd, "Q5POST %f", &val) == 1) {
        Route_SetQ5PostCm(val); BT_Printf("OK Q5POST=%.0f\r\n", Route_GetQ5PostCm()); return;
    }
    if (sscanf(cmd, "Q5STOP %d", &ival) == 1) {
        Route_SetQ5StopMs((uint16_t)ival); BT_Printf("OK Q5STOP=%u\r\n", Route_GetQ5StopMs()); return;
    }
    if (sscanf(cmd, "Q5TOUT %f", &val) == 1) {
        Route_SetQ5TimeoutMs((uint32_t)(val * 1000.0f)); BT_Printf("OK Q5TOUT=%.1f\r\n", Route_GetQ5TimeoutMs()/1000.0f); return;
    }
    if (sscanf(cmd, "Q5TMIN %f", &val) == 1) {
        Route_SetQ5DetectMinMs((uint32_t)(val * 1000.0f)); BT_Printf("OK Q5TMIN=%.1f\r\n", Route_GetQ5DetectMinMs()/1000.0f); return;
    }
    if (sscanf(cmd, "Q5TMAX %f", &val) == 1) {
        Route_SetQ5DetectMaxMs((uint32_t)(val * 1000.0f)); BT_Printf("OK Q5TMAX=%.1f\r\n", Route_GetQ5DetectMaxMs()/1000.0f); return;
    }

    /* 遥测 */
    if (sscanf(cmd, "TEL %d", &ival) == 1) {
        if (ival == 0) { g_tel_period_ms = 0; }
        else { if (ival < 100) ival = 100; if (ival > 2000) ival = 2000; g_tel_period_ms = (uint16_t)ival; }
        BT_Printf("OK TEL=%u\r\n", g_tel_period_ms); return;
    }

    BT_Printf("UNKNOWN [%s]\r\n", cmd);
}

/* ── 遥测数据输出（主循环调用） ── */
void BT_Telemetry(uint8_t gray_raw, int16_t speed_l, int16_t speed_r, float yaw)
{
    if (g_tel_period_ms == 0) return;
    BT_Printf("DATA T=%.2f D=%.1f RAW=0x%02X BC=%u P=%d S=%d VL=%d VR=%d Y=%.1f\r\n",
              (float)Route_GetElapsedMs() / 1000.0f,
              Route_GetDistanceCm(),
              gray_raw,
              (unsigned)Route_GetBlackMin(),  /* 显示阈值供参考 */
              Route_GetLineError(),
              Route_GetLineSteer(),
              speed_l, speed_r, yaw);
}

uint16_t BT_GetTelPeriod(void) { return g_tel_period_ms; }

/* ── 主循环调用 ── */
bool BT_Poll(void)
{
    /* 超时成帧 */
    if ((rx_ready == false) && (rx_idx > 0U) && (bt_tick != NULL) &&
        ((*bt_tick - rx_last_ms) > RX_TIMEOUT_MS)) {
        rx_buf[rx_idx] = '\0';
        rx_ready = true;
        rx_idx = 0U;
    }

    if (rx_ready) {
        rx_ready = false;
        trim_cmd(rx_buf);
        if (rx_buf[0] == '\0') return false;

        /* 防回显重复 */
        if (bt_tick != NULL && strcmp(rx_buf, last_cmd) == 0 &&
            (*bt_tick - last_cmd_ms) < 100U) {
            return false;
        }
        strncpy(last_cmd, rx_buf, RX_BUF_SIZE - 1);
        last_cmd[RX_BUF_SIZE - 1] = '\0';
        if (bt_tick != NULL) last_cmd_ms = *bt_tick;

        parse_cmd(rx_buf);
    }
    return false;
}
