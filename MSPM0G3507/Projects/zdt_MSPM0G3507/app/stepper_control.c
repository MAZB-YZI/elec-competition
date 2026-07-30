/**
 * @file stepper_control.c
 * @brief Stepper motor control application layer
 *
 * State transitions happen on ACK, not on command send.
 * Fault sequence: STOP → ACK/timeout → DISABLE → ACK/timeout → FAULT (latched).
 * User must call Stepper_ClearFault() to leave FAULT.
 */
#include "stepper_control.h"
#include "zdt_uart_port.h"
#include "zdt_emm_v5.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>

/* ---- Motor instance ---- */
static ZdtMotor g_motor;

/* ---- Application state ---- */
static ZdtState g_state          = ZDT_STATE_DISABLED;
static bool     g_homed          = false;
static uint8_t  g_fault_code     = 0;
static uint32_t g_consec_timeouts = 0;  /* Consecutive timeouts */
static uint32_t g_last_query_ms  = 0;

/* ---- Pending operation tracking ---- */
typedef enum {
    PENDING_NONE = 0,
    PENDING_ENABLE,
    PENDING_DISABLE,
    PENDING_STOP,
    PENDING_MOVE_ABS,   /* Absolute position — ACK sets moving_state=1 */
    PENDING_MOVE_VEL,   /* Velocity — stays MOVING until STOP */
    PENDING_HOME,
    PENDING_ZERO,
    PENDING_CLEAR_STALL,
    PENDING_QUERY,
} PendingOp;

static PendingOp g_pending_op = PENDING_NONE;

/* ---- Fault sequence state machine ---- */
typedef enum {
    FAULT_IDLE = 0,        /* Not in fault sequence */
    FAULT_STOP_SENT,       /* STOP sent, waiting for ACK/timeout */
    FAULT_DISABLE_SENT,    /* DISABLE sent, waiting for ACK/timeout */
    FAULT_DONE,            /* Fault sequence complete, latched in FAULT */
} FaultStep;

static FaultStep g_fault_step = FAULT_IDLE;

/* ---- Debug output ---- */
extern void UART_Debug_SendString(const char *str);
static void debug_log(const char *msg) { UART_Debug_SendString(msg); }

/* ========================================================================
 * Motor UART RX ISR
 * ======================================================================== */
void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_1_INST) == DL_UART_MAIN_IIDX_RX)
    {
        uint8_t byte = (uint8_t)DL_UART_Main_receiveData(UART_1_INST);
        ZDT_RxByte(&g_motor, byte);
    }
}

/* ========================================================================
 * Init
 * ======================================================================== */
void Stepper_Init(void)
{
    ZdtMotorConfig cfg = {
        .address     = 0x01,
        .write       = ZDT_Port_Transmit,
        .get_tick_ms = ZDT_Port_GetTickMs,
        .limit_min   = STEPPER_LIMIT_MIN,
        .limit_max   = STEPPER_LIMIT_MAX,
    };
    ZDT_Init(&g_motor, &cfg);

    g_state            = ZDT_STATE_DISABLED;
    g_homed            = false;
    g_fault_code       = 0;
    g_consec_timeouts  = 0;
    g_last_query_ms    = 0;
    g_pending_op       = PENDING_NONE;
    g_fault_step       = FAULT_IDLE;

    debug_log("[STEPPER] Init, state=DISABLED\r\n");
}

/* ========================================================================
 * Send command — bypasses pending check for STOP/DISABLE (fault commands)
 * ======================================================================== */
static bool send_cmd(uint8_t func, const uint8_t *params, uint8_t len,
                      PendingOp op, bool emergency)
{
    if (!emergency && g_motor.tx.result == ZDT_TX_PENDING) {
        return false;  /* Normal commands must wait */
    }

    /* Emergency commands: force-clear any pending transaction */
    if (emergency && g_motor.tx.result == ZDT_TX_PENDING) {
        g_motor.tx.result = ZDT_TX_IDLE;
    }

    if (!g_motor.cfg.write) return false;

    uint8_t frame[ZDT_MAX_FRAME_LEN];
    uint8_t flen = ZDT_MakeFrame(g_motor.cfg.address, func,
                                  params, len, frame);
    if (flen == 0) return false;

    g_motor.rx.frame_ready = false;

    if (!g_motor.cfg.write(frame, flen)) {
        return false;
    }

    g_motor.tx.sequence++;
    g_motor.tx.result = ZDT_TX_PENDING;
    g_motor.tx.expected_cmd = func;
    g_motor.tx.response_code = 0;
    g_motor.tx.deadline_ms = g_motor.cfg.get_tick_ms() + 50;
    g_pending_op = op;

    return true;
}

/* ========================================================================
 * Enter fault — start STOP → DISABLE sequence
 * Idempotent: does nothing if already in fault sequence
 * ======================================================================== */
static void enter_fault(uint8_t code)
{
    if (g_state == ZDT_STATE_FAULT) return;

    g_fault_code = code;
    g_state = ZDT_STATE_FAULT;
    g_fault_step = FAULT_IDLE;

    /* Send STOP (emergency, bypasses pending) */
    uint8_t stop_param = 0x98;
    if (send_cmd(ZDT_CMD_STOP, &stop_param, 1, PENDING_STOP, true)) {
        g_fault_step = FAULT_STOP_SENT;
        debug_log("[STEPPER] FAULT: STOP sent\r\n");
    } else {
        /* Can't even send STOP — skip to DISABLE */
        uint8_t dis_params[3] = { 0xAB, 0x00, 0x00 };
        if (send_cmd(ZDT_CMD_ENABLE, dis_params, 3, PENDING_DISABLE, true)) {
            g_fault_step = FAULT_DISABLE_SENT;
            debug_log("[STEPPER] FAULT: DISABLE sent (STOP failed)\r\n");
        } else {
            g_fault_step = FAULT_DONE;
            debug_log("[STEPPER] FAULT: comms dead, latched\r\n");
        }
    }
}

/* ========================================================================
 * Advance fault sequence after STOP completes (ACK or timeout)
 * ======================================================================== */
static void fault_advance_after_stop(void)
{
    /* Send DISABLE (emergency) */
    uint8_t dis_params[3] = { 0xAB, 0x00, 0x00 };
    if (send_cmd(ZDT_CMD_ENABLE, dis_params, 3, PENDING_DISABLE, true)) {
        g_fault_step = FAULT_DISABLE_SENT;
        debug_log("[STEPPER] FAULT: DISABLE sent\r\n");
    } else {
        g_fault_step = FAULT_DONE;
        debug_log("[STEPPER] FAULT: DISABLE send failed, latched\r\n");
    }
}

/* ========================================================================
 * Handle transaction result — returns true if something was processed
 * ======================================================================== */
static bool handle_tx_result(void)
{
    ZdtTransactionResult res = g_motor.tx.result;

    if (res == ZDT_TX_IDLE || res == ZDT_TX_PENDING) {
        return false;
    }

    /* Consume the result */
    g_motor.tx.result = ZDT_TX_IDLE;
    PendingOp op = g_pending_op;
    g_pending_op = PENDING_NONE;

    /* ---- TIMEOUT ---- */
    if (res == ZDT_TX_TIMEOUT) {
        g_consec_timeouts++;

        char buf[64];
        snprintf(buf, sizeof(buf),
                 "[STEPPER] Timeout #%lu (op=%d)\r\n",
                 (unsigned long)g_consec_timeouts, op);
        debug_log(buf);

        /* Fault/stop sequence: advance on timeout */
        if (g_fault_step == FAULT_STOP_SENT && op == PENDING_STOP) {
            if (g_state == ZDT_STATE_FAULT) {
                fault_advance_after_stop();  /* Fault: send DISABLE, stay FAULT */
            } else {
                /* Normal stop: STOP timed out, still try DISABLE */
                g_fault_step = FAULT_IDLE;
                uint8_t dis_params[3] = { 0xAB, 0x00, 0x00 };
                send_cmd(ZDT_CMD_ENABLE, dis_params, 3, PENDING_DISABLE, true);
                debug_log("[STEPPER] Stop timeout, sending DISABLE\r\n");
            }
            return true;
        }
        if (g_fault_step == FAULT_DISABLE_SENT && op == PENDING_DISABLE) {
            if (g_state == ZDT_STATE_FAULT) {
                g_fault_step = FAULT_DONE;
                debug_log("[STEPPER] FAULT: DISABLE timeout, latched\r\n");
            } else {
                g_fault_step = FAULT_IDLE;
                g_state = ZDT_STATE_DISABLED;
                debug_log("[STEPPER] DISABLE timeout -> DISABLED\r\n");
            }
            return true;
        }

        /* Normal operation: too many consecutive timeouts → FAULT */
        if (g_state != ZDT_STATE_FAULT &&
            g_consec_timeouts >= STEPPER_MAX_TIMEOUTS) {
            enter_fault(STEPPER_ERR_TIMEOUT);
        }
        return true;
    }

    /* ---- NACK ---- */
    if (res == ZDT_TX_NACK) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "[STEPPER] NACK 0x%02X (op=%d)\r\n",
                 g_motor.tx.response_code, op);
        debug_log(buf);

        if (op == PENDING_ENABLE && g_state != ZDT_STATE_FAULT) {
            g_state = ZDT_STATE_DISABLED;
        }

        if (g_state == ZDT_STATE_FAULT) {
            /* Fault sequence: NACK on STOP → try DISABLE anyway */
            if (g_fault_step == FAULT_STOP_SENT && op == PENDING_STOP) {
                fault_advance_after_stop();
            }
            /* NACK on DISABLE during fault → latch */
            if (g_fault_step == FAULT_DISABLE_SENT && op == PENDING_DISABLE) {
                g_fault_step = FAULT_DONE;
                debug_log("[STEPPER] FAULT: DISABLE NACK, latched\r\n");
            }
        } else {
            /* Normal stop sequence: NACK on STOP → just go DISABLED */
            if (g_fault_step == FAULT_STOP_SENT && op == PENDING_STOP) {
                g_fault_step = FAULT_IDLE;
                g_state = ZDT_STATE_DISABLED;
                debug_log("[STEPPER] Stop NACK -> DISABLED\r\n");
            }
        }
        return true;
    }

    /* ---- ACK ---- */
    g_consec_timeouts = 0;  /* Valid ACK resets consecutive counter */

    /* Fault sequence: handle ACK only when actually in FAULT state */
    if (g_state == ZDT_STATE_FAULT) {
        if (g_fault_step == FAULT_STOP_SENT && op == PENDING_STOP) {
            fault_advance_after_stop();
            return true;
        }
        if (g_fault_step == FAULT_DISABLE_SENT && op == PENDING_DISABLE) {
            g_fault_step = FAULT_DONE;
            debug_log("[STEPPER] FAULT: DISABLED, latched. Send CLEAR to recover.\r\n");
            return true;
        }
    }

    /* Normal stop sequence (not fault): STOP ACK → send DISABLE */
    if (g_fault_step == FAULT_STOP_SENT && op == PENDING_STOP) {
        g_fault_step = FAULT_IDLE;
        uint8_t dis_params[3] = { 0xAB, 0x00, 0x00 };
        if (send_cmd(ZDT_CMD_ENABLE, dis_params, 3, PENDING_DISABLE, true)) {
            debug_log("[STEPPER] Stop ACK, sending DISABLE\r\n");
        } else {
            g_state = ZDT_STATE_DISABLED;
            debug_log("[STEPPER] Stop ACK, DISABLE send failed\r\n");
        }
        return true;
    }

    /* Normal state transitions on ACK */
    switch (op) {
    case PENDING_ENABLE:
        g_state = ZDT_STATE_READY;
        debug_log("[STEPPER] Enabled -> READY\r\n");
        break;

    case PENDING_DISABLE:
        g_state = ZDT_STATE_DISABLED;
        debug_log("[STEPPER] Disabled\r\n");
        break;

    case PENDING_STOP:
        /* Normal stop (not during fault sequence) */
        g_state = ZDT_STATE_DISABLED;
        debug_log("[STEPPER] Stopped -> DISABLED\r\n");
        break;

    case PENDING_MOVE_ABS:
        g_state = ZDT_STATE_MOVING;
        g_motor.moving_state = 1;  /* Enable position-reached detection */
        break;

    case PENDING_MOVE_VEL:
        g_state = ZDT_STATE_MOVING;
        g_motor.moving_state = 2;  /* Velocity mode — stays MOVING until STOP */
        break;

    case PENDING_HOME:
        g_state = ZDT_STATE_HOMING;
        debug_log("[STEPPER] Homing -> HOMING\r\n");
        break;

    case PENDING_ZERO:
        g_motor.position = 0;
        g_homed = true;
        debug_log("[STEPPER] Zero set, homed=true\r\n");
        break;

    case PENDING_CLEAR_STALL:
        debug_log("[STEPPER] Stall cleared\r\n");
        break;

    case PENDING_QUERY:
    case PENDING_NONE:
    default:
        break;
    }

    return true;
}

/* ========================================================================
 * Public commands
 * ======================================================================== */

uint8_t Stepper_Enable(void)
{
    if (g_state == ZDT_STATE_FAULT) return STEPPER_ERR_FAULT;
    if (g_pending_op != PENDING_NONE) return STEPPER_ERR_BUSY;

    uint8_t params[3] = { 0xAB, 0x01, 0x00 };
    if (send_cmd(ZDT_CMD_ENABLE, params, 3, PENDING_ENABLE, false)) {
        return STEPPER_OK;
    }
    return STEPPER_ERR_TIMEOUT;
}

uint8_t Stepper_Disable(void)
{
    uint8_t params[3] = { 0xAB, 0x00, 0x00 };
    if (send_cmd(ZDT_CMD_ENABLE, params, 3, PENDING_DISABLE, false)) {
        return STEPPER_OK;
    }
    return STEPPER_ERR_BUSY;
}

void Stepper_StopNow(void)
{
    /* Normal stop: STOP → DISABLE → DISABLED (not FAULT) */
    uint8_t stop_param = 0x98;
    if (send_cmd(ZDT_CMD_STOP, &stop_param, 1, PENDING_STOP, true)) {
        g_fault_step = FAULT_STOP_SENT;
        g_state = ZDT_STATE_DISABLED;
        debug_log("[STEPPER] Stop sent\r\n");
    } else {
        /* Can't send STOP — just force DISABLED */
        g_state = ZDT_STATE_DISABLED;
        g_pending_op = PENDING_NONE;
        debug_log("[STEPPER] Stop failed, forced DISABLED\r\n");
    }
}

uint8_t Stepper_MoveAbsolute(int32_t target_position,
                              uint16_t rpm, uint8_t acceleration)
{
    if (g_state == ZDT_STATE_FAULT) return STEPPER_ERR_FAULT;
    if (g_state == ZDT_STATE_DISABLED) return STEPPER_ERR_FAULT;
    if (g_pending_op != PENDING_NONE) return STEPPER_ERR_BUSY;

    ZdtResult res = ZDT_CheckLimits(&g_motor, target_position);
    if (res != ZDT_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "[STEPPER] Soft limit: target=%ld range=[%ld,%ld]\r\n",
                 (long)target_position,
                 (long)g_motor.cfg.limit_min,
                 (long)g_motor.cfg.limit_max);
        debug_log(buf);
        return STEPPER_ERR_SOFT_LIMIT;
    }

    /* Build position params directly */
    uint8_t dir;
    uint32_t magnitude;
    if (target_position >= 0) {
        dir = ZDT_DIR_CW;
        magnitude = (uint32_t)target_position;
    } else {
        dir = ZDT_DIR_CCW;
        magnitude = (uint32_t)(-(int64_t)target_position);
    }
    if (rpm > 5000) rpm = 5000;
    if (rpm == 0) rpm = 1;

    uint8_t params[10];
    params[0] = dir;
    params[1] = (uint8_t)(rpm >> 8);
    params[2] = (uint8_t)(rpm & 0xFF);
    params[3] = acceleration;
    params[4] = (uint8_t)((magnitude >> 24) & 0xFF);
    params[5] = (uint8_t)((magnitude >> 16) & 0xFF);
    params[6] = (uint8_t)((magnitude >> 8) & 0xFF);
    params[7] = (uint8_t)(magnitude & 0xFF);
    params[8] = ZDT_POS_MODE_ABSOLUTE;
    params[9] = 0x00;

    if (send_cmd(ZDT_CMD_POSITION, params, 10, PENDING_MOVE_ABS, false)) {
        g_motor.target_position = target_position;
        return STEPPER_OK;
    }
    return STEPPER_ERR_TIMEOUT;
}

uint8_t Stepper_MoveVelocity(int16_t rpm, uint8_t acceleration)
{
    if (g_state == ZDT_STATE_FAULT || g_state == ZDT_STATE_DISABLED)
        return STEPPER_ERR_FAULT;
    if (g_pending_op != PENDING_NONE) return STEPPER_ERR_BUSY;

    uint8_t dir = (rpm >= 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    uint16_t speed = (rpm >= 0) ? (uint16_t)rpm : (uint16_t)(-rpm);
    if (speed > 5000) speed = 5000;

    uint8_t params[5];
    params[0] = dir;
    params[1] = (uint8_t)(speed >> 8);
    params[2] = (uint8_t)(speed & 0xFF);
    params[3] = acceleration;
    params[4] = 0x00;

    if (send_cmd(ZDT_CMD_VELOCITY, params, 5, PENDING_MOVE_VEL, false)) {
        return STEPPER_OK;
    }
    return STEPPER_ERR_TIMEOUT;
}

uint8_t Stepper_Home(ZdtHomeMode mode)
{
    if (g_state == ZDT_STATE_FAULT) return STEPPER_ERR_FAULT;
    if (g_pending_op != PENDING_NONE) return STEPPER_ERR_BUSY;

    uint8_t params[2] = { (uint8_t)mode, 0x00 };
    if (send_cmd(ZDT_CMD_HOME, params, 2, PENDING_HOME, false)) {
        g_motor.moving_state = 3;
        return STEPPER_OK;
    }
    return STEPPER_ERR_TIMEOUT;
}

uint8_t Stepper_SetZero(void)
{
    if (g_pending_op != PENDING_NONE) return STEPPER_ERR_BUSY;

    uint8_t params[1] = { 0x6D };
    if (send_cmd(ZDT_CMD_SET_ZERO, params, 1, PENDING_ZERO, false)) {
        return STEPPER_OK;
    }
    return STEPPER_ERR_TIMEOUT;
}

uint8_t Stepper_SaveZero(void)
{
    if (g_pending_op != PENDING_NONE) return STEPPER_ERR_BUSY;

    uint8_t params[2] = { 0x88, 0x01 };
    if (send_cmd(0x93, params, 2, PENDING_ZERO, false)) {
        return STEPPER_OK;
    }
    return STEPPER_ERR_TIMEOUT;
}

uint8_t Stepper_QueryPosition(void)
{
    if (!send_cmd(ZDT_CMD_READ_POSITION, NULL, 0, PENDING_QUERY, false)) {
        return STEPPER_ERR_TIMEOUT;
    }
    return STEPPER_OK;
}

/* ========================================================================
 * Poll — call from main loop
 * ======================================================================== */
void Stepper_Poll(void)
{
    uint32_t now = ZDT_Port_GetTickMs();

    /* 1. Protocol layer: process frames + check timeouts */
    ZDT_Poll(&g_motor);

    /* 2. Handle transaction results (ACK/NACK/TIMEOUT) */
    handle_tx_result();

    /* 3. State-specific periodic queries */
    switch (g_state) {
    case ZDT_STATE_READY:
        if (g_pending_op == PENDING_NONE &&
            (int32_t)(now - g_last_query_ms) >= (int32_t)STEPPER_QUERY_INTERVAL_MS)
        {
            g_last_query_ms = now;
            if (ZDT_RequestPosition(&g_motor)) {
                g_pending_op = PENDING_QUERY;
            }
        }
        break;

    case ZDT_STATE_MOVING:
        if (g_pending_op == PENDING_NONE &&
            (int32_t)(now - g_last_query_ms) >= (int32_t)STEPPER_QUERY_INTERVAL_MS)
        {
            g_last_query_ms = now;
            if (ZDT_RequestPosition(&g_motor)) {
                g_pending_op = PENDING_QUERY;
            }
        }
        /* Check if position reached */
        if (g_motor.moving_state == 1) {
            int32_t error = g_motor.target_position - g_motor.position;
            if (error < 0) error = -error;
            if (error <= STEPPER_POS_TOLERANCE) {
                g_motor.moving_state = 0;
                g_state = ZDT_STATE_READY;
                debug_log("[STEPPER] Position reached -> READY\r\n");
            }
        }
        break;

    case ZDT_STATE_HOMING:
        if (g_pending_op == PENDING_NONE &&
            (int32_t)(now - g_last_query_ms) >= (int32_t)STEPPER_QUERY_INTERVAL_MS)
        {
            g_last_query_ms = now;
            if (ZDT_RequestHomeStatus(&g_motor)) {
                g_pending_op = PENDING_QUERY;
            }
            /* Check homing completion (after response arrives) */
            if (g_motor.home_status == 2) {
                g_motor.moving_state = 0;
                g_homed = true;
                g_state = ZDT_STATE_READY;
                debug_log("[STEPPER] Homing complete -> READY\r\n");
            }
        }
        break;

    case ZDT_STATE_FAULT:
        /* Latched — fault sequence handled in handle_tx_result */
        break;

    case ZDT_STATE_DISABLED:
    default:
        break;
    }
}

/* ========================================================================
 * Clear fault — pure software: FAULT → DISABLED, no protocol commands
 * ======================================================================== */
uint8_t Stepper_ClearFault(void)
{
    if (g_state != ZDT_STATE_FAULT) return STEPPER_OK;

    g_consec_timeouts = 0;
    g_pending_op = PENDING_NONE;
    g_fault_step = FAULT_IDLE;
    g_fault_code = 0;
    g_state = ZDT_STATE_DISABLED;

    debug_log("[STEPPER] Fault cleared -> DISABLED\r\n");
    return STEPPER_OK;
}

/* ========================================================================
 * Clear stall protection — separate command, goes through tx management
 * ======================================================================== */
uint8_t Stepper_ClearStall(void)
{
    if (g_pending_op != PENDING_NONE) return STEPPER_ERR_BUSY;

    uint8_t params[1] = { 0x52 };
    if (send_cmd(ZDT_CMD_CLEAR_STALL, params, 1, PENDING_CLEAR_STALL, false)) {
        return STEPPER_OK;
    }
    return STEPPER_ERR_TIMEOUT;
}

/* ========================================================================
 * Query position error — through tx management
 * ======================================================================== */
uint8_t Stepper_QueryPositionError(void)
{
    if (!send_cmd(ZDT_CMD_READ_POS_ERR, NULL, 0, PENDING_QUERY, false)) {
        return STEPPER_ERR_BUSY;
    }
    return STEPPER_OK;
}

/* ========================================================================
 * Status getters
 * ======================================================================== */
ZdtState  Stepper_GetState(void)           { return g_state; }
int32_t   Stepper_GetPosition(void)        { return g_motor.position; }
int32_t   Stepper_GetPositionError(void)   { return g_motor.position_error; }
int16_t   Stepper_GetSpeed(void)           { return g_motor.speed; }
uint32_t  Stepper_GetTimeoutCount(void)    { return g_consec_timeouts; }
uint8_t   Stepper_GetFaultCode(void)       { return g_fault_code; }
bool      Stepper_IsFault(void)            { return g_state == ZDT_STATE_FAULT; }
bool      Stepper_IsHomed(void)            { return g_homed; }

const char *Stepper_StateName(ZdtState state)
{
    switch (state) {
    case ZDT_STATE_DISABLED: return "DISABLED";
    case ZDT_STATE_READY:    return "READY";
    case ZDT_STATE_MOVING:   return "MOVING";
    case ZDT_STATE_HOMING:   return "HOMING";
    case ZDT_STATE_FAULT:    return "FAULT";
    default:                 return "UNKNOWN";
    }
}

void Stepper_SetLimits(int32_t min_pos, int32_t max_pos)
{
    g_motor.cfg.limit_min = min_pos;
    g_motor.cfg.limit_max = max_pos;
}

ZdtMotor *Stepper_GetMotor(void) { return &g_motor; }
