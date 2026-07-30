/**
 * @file empty.c
 * @brief ZDT Emm V5 stepper motor test — main entry point
 *
 * Architecture:
 *   Protocol layer:  zdt_emm_v5.c/h   — frame encode/decode
 *   BSP layer:       zdt_uart_port.c/h — MSPM0 UART + tick
 *   App layer:       stepper_control.c/h — state machine + safety
 *   Calibration:     linkage_calibration.c/h — angle ↔ position
 *
 * UART: UART3 peripheral on PA23(TX)/PA24(RX) — motor TTL
 * Debug: UART0 peripheral on PA10(TX)/PA11(RX) — serial console
 *
 * Safety:
 *   - Software limits: -650 ~ +650 pulses
 *   - Communication timeout: 3 consecutive → FAULT → stop + disable
 *   - No automatic high-speed rotation on power-up
 */
#include "ti_msp_dl_config.h"
#include "stepper_control.h"
#include "linkage_calibration.h"
#include "zdt_uart_port.h"
#include "uart_debug.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ---- Test parameters ---- */
#define TEST_RPM          100   /* Movement speed for tests */
#define TEST_ACCELERATION  20   /* Acceleration for tests */

/* ---- State for serial command processing ---- */
static char     g_cmd_buf[64];
static uint8_t  g_cmd_idx    = 0;
static volatile uint8_t g_cmd_ready = 0;

/* ---- Forward declarations ---- */
static void process_serial_cmd(const char *cmd);
static void print_status(void);

/* ---- Debug UART RX interrupt handler ---- */
void UART_DEBUG_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST) == DL_UART_MAIN_IIDX_RX)
    {
        uint8_t ch = (uint8_t)DL_UART_Main_receiveData(UART_DEBUG_INST);

        if (ch == '\n' || ch == '\r') {
            if (g_cmd_idx > 0) {
                g_cmd_buf[g_cmd_idx] = '\0';
                g_cmd_ready = 1;
            }
        } else if (g_cmd_idx < sizeof(g_cmd_buf) - 1) {
            g_cmd_buf[g_cmd_idx++] = ch;
        }
    }
}

/* ---- Timer interrupt (LED heartbeat) ---- */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        /* Toggle LED to show system is alive */
        DL_GPIO_togglePins(LED_PORT, LED_led_PIN);
    }
}

int main(void)
{
    /* 1. System initialization (clock, GPIO, UART, timer) */
    SYSCFG_DL_init();

    /* 2. Initialize BSP (motor UART, SysTick) */
    ZDT_Port_Init();

    /* 3. Enable debug UART */
    DL_UART_Main_enableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    /* 4. Enable timer interrupt (LED blink) */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* 5. Initialize calibration table */
    Linkage_Init();

    /* 6. Initialize stepper control (state machine, limits, etc.) */
    Stepper_Init();

    /* 7. Send startup banner */
    UART_Debug_SendString("\r\n========================================\r\n");
    UART_Debug_SendString("ZDT Emm V5 Stepper Motor Control\r\n");
    UART_Debug_SendString("PA23=TX  PA24=RX  115200 8N1\r\n");
    UART_Debug_SendString("========================================\r\n");
    UART_Debug_SendString("Commands:\r\n");
    UART_Debug_SendString("  ENABLE      - Enable motor\r\n");
    UART_Debug_SendString("  DISABLE     - Disable motor\r\n");
    UART_Debug_SendString("  STOP        - Emergency stop\r\n");
    UART_Debug_SendString("  HOME        - Start homing\r\n");
    UART_Debug_SendString("  ZERO        - Set current pos as zero (temp)\r\n");
    UART_Debug_SendString("  SAVEZERO    - Save zero to Flash (permanent)\r\n");
    UART_Debug_SendString("  STATUS      - Show current state\r\n");
    UART_Debug_SendString("  POS<n>      - Move to position n (pulses)\r\n");
    UART_Debug_SendString("  DEG<n>      - Move to n degrees (direct)\r\n");
    UART_Debug_SendString("  ANGLE<n>    - Move to n millideg (calib)\r\n");
    UART_Debug_SendString("  VEL<n>      - Move at velocity n RPM\r\n");
    UART_Debug_SendString("  READPOS     - Read real-time position\r\n");
    UART_Debug_SendString("  READERR     - Read position error\r\n");
    UART_Debug_SendString("  TESTCW      - CW 50 RPM test\r\n");
    UART_Debug_SendString("  TESTCCW     - CCW 50 RPM test\r\n");
    UART_Debug_SendString("  CLEAR       - Clear fault\r\n");
    UART_Debug_SendString("  CLEARSTALL  - Clear stall protect\r\n");
    UART_Debug_SendString("========================================\r\n");
    UART_Debug_SendString("State: DISABLED (send ENABLE first)\r\n");

    /* 8. Main loop */
    while (1) {
        /* Process serial commands from debug UART */
        if (g_cmd_ready) {
            process_serial_cmd(g_cmd_buf);
            g_cmd_idx = 0;
            g_cmd_ready = 0;
        }

        /* Poll stepper state machine (handles responses, timeouts, queries) */
        Stepper_Poll();
    }
}

/* ---- Serial command processor ---- */
static void process_serial_cmd(const char *cmd)
{
    char buf[80];

    if (strcmp(cmd, "ENABLE") == 0) {
        uint8_t res = Stepper_Enable();
        snprintf(buf, sizeof(buf), "Enable: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strcmp(cmd, "DISABLE") == 0) {
        uint8_t res = Stepper_Disable();
        snprintf(buf, sizeof(buf), "Disable: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strcmp(cmd, "STOP") == 0) {
        Stepper_StopNow();
        UART_Debug_SendString("Emergency stop\r\n");

    } else if (strcmp(cmd, "HOME") == 0) {
        uint8_t res = Stepper_Home(ZDT_HOME_SINGLE_NEAREST);
        snprintf(buf, sizeof(buf), "Home: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strcmp(cmd, "ZERO") == 0) {
        uint8_t res = Stepper_SetZero();
        snprintf(buf, sizeof(buf), "Set zero: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strcmp(cmd, "SAVEZERO") == 0) {
        uint8_t res = Stepper_SaveZero();
        snprintf(buf, sizeof(buf), "Save zero to Flash: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strcmp(cmd, "STATUS") == 0) {
        print_status();

    } else if (strcmp(cmd, "CLEAR") == 0) {
        uint8_t res = Stepper_ClearFault();
        snprintf(buf, sizeof(buf), "Clear fault: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strcmp(cmd, "CLEARSTALL") == 0) {
        uint8_t res = Stepper_ClearStall();
        snprintf(buf, sizeof(buf), "Clear stall: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    /* ---- 位置命令 ---- */

    } else if (strncmp(cmd, "POS", 3) == 0) {
        int32_t target = atol(&cmd[3]);
        uint8_t res = Stepper_MoveAbsolute(target, TEST_RPM, TEST_ACCELERATION);
        snprintf(buf, sizeof(buf), "Move to %ld: %s\r\n",
                 (long)target,
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strncmp(cmd, "DEG", 3) == 0) {
        /* DEG<n> — n is degrees (supports decimals like DEG22.5, DEG-67.5)
         * Uses millidegrees internally: mdeg = round(deg * 1000)
         * pulses = mdeg * 3200 / 360000 */
        float deg_f = atof(&cmd[3]);
        int32_t millideg = (int32_t)(deg_f * 1000.0f + (deg_f >= 0 ? 0.5f : -0.5f));
        int32_t position = (int32_t)((int64_t)millideg * 3200L / 360000L);
        uint8_t res = Stepper_MoveAbsolute(position, TEST_RPM, TEST_ACCELERATION);
        int32_t abs_mdeg = millideg < 0 ? -millideg : millideg;
        snprintf(buf, sizeof(buf), "DEG %c%ld.%03d -> pos %ld: %s\r\n",
                 millideg < 0 ? '-' : '+',
                 (long)(abs_mdeg / 1000), (int)(abs_mdeg % 1000),
                 (long)position,
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strncmp(cmd, "ANGLE", 5) == 0) {
        /* ANGLE<millideg> — through calibration table (NOT CALIBRATED YET) */
        UART_Debug_SendString("WARNING: ANGLE uses example table, not calibrated!\r\n");
        int16_t angle_mdeg = (int16_t)atoi(&cmd[5]);
        int32_t position;
        if (!Linkage_AngleToPosition(angle_mdeg, &position)) {
            snprintf(buf, sizeof(buf), "ANGLE %d mdeg: OUT OF RANGE\r\n", angle_mdeg);
            UART_Debug_SendString(buf);
        } else {
            uint8_t res = Stepper_MoveAbsolute(position, TEST_RPM, TEST_ACCELERATION);
            snprintf(buf, sizeof(buf), "ANGLE %d mdeg -> pos %ld: %s\r\n",
                     angle_mdeg, (long)position,
                     res == STEPPER_OK ? "OK" : "FAIL");
            UART_Debug_SendString(buf);
        }

    } else if (strncmp(cmd, "VEL", 3) == 0) {
        int16_t rpm = (int16_t)atoi(&cmd[3]);
        uint8_t res = Stepper_MoveVelocity(rpm, TEST_ACCELERATION);
        snprintf(buf, sizeof(buf), "Velocity %d RPM: %s\r\n",
                 rpm,
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    /* ---- 查询命令 ---- */

    } else if (strcmp(cmd, "READPOS") == 0) {
        Stepper_QueryPosition();
        UART_Debug_SendString("Position query sent (check STATUS later)\r\n");

    } else if (strcmp(cmd, "READERR") == 0) {
        uint8_t res = Stepper_QueryPositionError();
        snprintf(buf, sizeof(buf), "Error query: %s\r\n",
                 res == STEPPER_OK ? "sent" : "FAIL");
        UART_Debug_SendString(buf);

    /* ---- 调试测试命令 ---- */

    } else if (strcmp(cmd, "TESTCW") == 0) {
        uint8_t res = Stepper_MoveVelocity(50, 20);
        snprintf(buf, sizeof(buf), "Test CW 50 RPM: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else if (strcmp(cmd, "TESTCCW") == 0) {
        uint8_t res = Stepper_MoveVelocity(-50, 20);
        snprintf(buf, sizeof(buf), "Test CCW 50 RPM: %s\r\n",
                 res == STEPPER_OK ? "OK" : "FAIL");
        UART_Debug_SendString(buf);

    } else {
        UART_Debug_SendString("Unknown command\r\n");
    }
}

/* ---- Print motor status ---- */
/* Convert pulses to degrees: pos * 360 / 3200, displayed as XX.XX */
static void format_degrees(char *buf, size_t len, int32_t pulses)
{
    int32_t centideg = pulses * 36000L / 3200L;  /* hundredths of degrees */
    int32_t abs_cd = centideg < 0 ? -centideg : centideg;
    snprintf(buf, len, "%c%ld.%02d",
             centideg < 0 ? '-' : '+',
             (long)(abs_cd / 100), (int)(abs_cd % 100));
}

static void print_status(void)
{
    char buf[120];
    char pos_str[16], err_str[16];

    int32_t pos = Stepper_GetPosition();
    int32_t err = Stepper_GetPositionError();
    format_degrees(pos_str, sizeof(pos_str), pos);
    format_degrees(err_str, sizeof(err_str), err);

    snprintf(buf, sizeof(buf),
             "State: %s  Motor: %s deg  Err: %s deg  RPM: %d\r\n",
             Stepper_StateName(Stepper_GetState()),
             pos_str, err_str, Stepper_GetSpeed());
    UART_Debug_SendString(buf);

    snprintf(buf, sizeof(buf),
             "Homed: %s  Timeouts: %lu  Fault: %u\r\n",
             Stepper_IsHomed() ? "YES" : "NO",
             (unsigned long)Stepper_GetTimeoutCount(),
             Stepper_GetFaultCode());
    UART_Debug_SendString(buf);

    int16_t angle;
    if (Linkage_PositionToAngle(pos, &angle)) {
        snprintf(buf, sizeof(buf),
                 "Linkage: %d.%03d deg (example, NOT calibrated)\r\n",
                 angle / 1000, (angle < 0 ? -angle : angle) % 1000);
    } else {
        snprintf(buf, sizeof(buf), "Linkage angle: OUT OF RANGE\r\n");
    }
    UART_Debug_SendString(buf);
}
