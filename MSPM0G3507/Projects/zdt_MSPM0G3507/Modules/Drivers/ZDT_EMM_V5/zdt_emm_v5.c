/**
 * @file zdt_emm_v5.c
 * @brief ZDT Emm V5 stepper driver protocol implementation
 *
 * Frame format: [ADDR] [FUNC] [PARAMS...] [0x6B]
 * Query responses: [ADDR] [FUNC] [data...] [0x6B]
 *
 * Position response (0x36): dir(1) + magnitude(4) = 5 data bytes
 * Speed response   (0x35): dir(1) + speed(2) = 3 data bytes (NOT 4)
 * Pos error (0x37):        dir(1) + magnitude(4) = 5 data bytes
 * Status  (0x3A):          flags(1) = 1 data byte
 * Home st (0x3B):          status(1) = 1 data byte
 *
 * ACK frames (control commands): response(1) = 1 data byte
 *   0x02 = success, 0xE2 = condition not met, 0xEE = command error
 */
#include "zdt_emm_v5.h"
#include <string.h>

/* ========================================================================
 * Frame assembly
 * ======================================================================== */

uint8_t ZDT_MakeFrame(uint8_t addr, uint8_t func,
                       const uint8_t *params, uint8_t param_len,
                       uint8_t *out_buf)
{
    if (!out_buf || param_len > ZDT_MAX_DATA_LEN) return 0;

    uint8_t idx = 0;
    out_buf[idx++] = addr;
    out_buf[idx++] = func;
    if (params && param_len > 0) {
        memcpy(&out_buf[idx], params, param_len);
        idx += param_len;
    }
    out_buf[idx++] = ZDT_FRAME_TAIL;
    return idx;
}

/* ========================================================================
 * Init
 * ======================================================================== */

void ZDT_Init(ZdtMotor *motor, const ZdtMotorConfig *cfg)
{
    if (!motor || !cfg) return;
    memset(motor, 0, sizeof(ZdtMotor));
    motor->cfg = *cfg;
    motor->rx.state = ZDT_PARSE_IDLE;
    motor->tx.result = ZDT_TX_IDLE;
}

/* ========================================================================
 * RX byte — call from UART ISR
 * ======================================================================== */

/**
 * Expected data byte count after func code, BEFORE 0x6B tail.
 *
 * ACK commands (enable/stop/vel/pos/home/zero/clear):
 *   1 byte: response code (0x02/0xE2/0xEE)
 *
 * 0x35 speed:      dir(1) + speed_H(1) + speed_L(1) = 3 bytes
 * 0x36 position:   dir(1) + pos(4) = 5 bytes
 * 0x37 pos error:  dir(1) + err(4) = 5 bytes
 * 0x3A status:     flags(1) = 1 byte
 * 0x3B home st:    status(1) = 1 byte
 */
static uint8_t get_expected_data_len(uint8_t func)
{
    switch (func) {
    case ZDT_CMD_READ_SPEED:     return 3;  /* dir + 2 bytes speed */
    case ZDT_CMD_READ_POSITION:  return 5;  /* dir + 4 bytes position */
    case ZDT_CMD_READ_POS_ERR:   return 5;  /* dir + 4 bytes error */
    case ZDT_CMD_READ_STATUS:    return 1;  /* 1 byte flags */
    case ZDT_CMD_READ_HOME_ST:   return 1;  /* 1 byte status */
    default:                     return 1;  /* ACK: 1 byte response */
    }
}

void ZDT_RxByte(ZdtMotor *motor, uint8_t byte)
{
    if (!motor) return;
    ZdtRxState *rx = &motor->rx;

    switch (rx->state) {
    case ZDT_PARSE_IDLE:
        if (byte == motor->cfg.address) {
            rx->address = byte;
            rx->state = ZDT_PARSE_ADDR;
        }
        break;

    case ZDT_PARSE_ADDR:
        rx->func_code = byte;
        rx->data_idx = 0;
        rx->expected_len = get_expected_data_len(byte);
        rx->state = ZDT_PARSE_FUNC;
        break;

    case ZDT_PARSE_FUNC:
        if (rx->data_idx < ZDT_MAX_DATA_LEN) {
            rx->data[rx->data_idx++] = byte;
        }
        if (rx->data_idx >= rx->expected_len) {
            rx->state = ZDT_PARSE_DATA;
        }
        break;

    case ZDT_PARSE_DATA:
        if (byte == ZDT_FRAME_TAIL) {
            rx->frame_ready = true;
        }
        rx->state = ZDT_PARSE_IDLE;
        break;

    default:
        rx->state = ZDT_PARSE_IDLE;
        break;
    }
}

/* ========================================================================
 * Internal send — rejects if previous transaction still pending
 * ======================================================================== */

static bool send_command(ZdtMotor *motor, uint8_t func,
                          const uint8_t *params, uint8_t param_len)
{
    if (!motor || !motor->cfg.write) return false;

    /* Reject if previous transaction not yet completed */
    if (motor->tx.result == ZDT_TX_PENDING) {
        return false;
    }

    uint8_t frame[ZDT_MAX_FRAME_LEN];
    uint8_t len = ZDT_MakeFrame(motor->cfg.address, func,
                                 params, param_len, frame);
    if (len == 0) return false;

    /* Clear old response state */
    motor->rx.frame_ready = false;

    /* Send frame */
    if (!motor->cfg.write(frame, len)) {
        return false;
    }

    /* Set up transaction */
    motor->tx.sequence++;
    motor->tx.result = ZDT_TX_PENDING;
    motor->tx.expected_cmd = func;
    motor->tx.response_code = 0;
    motor->tx.deadline_ms = motor->cfg.get_tick_ms() + 50;

    return true;
}

/* ========================================================================
 * Command functions
 * ======================================================================== */

bool ZDT_Enable(ZdtMotor *motor, bool enable)
{
    uint8_t params[3] = { 0xAB, enable ? 0x01 : 0x00, 0x00 };
    return send_command(motor, ZDT_CMD_ENABLE, params, 3);
}

bool ZDT_Stop(ZdtMotor *motor)
{
    uint8_t params[1] = { 0x98 };
    return send_command(motor, ZDT_CMD_STOP, params, 1);
}

bool ZDT_ClearStallProtect(ZdtMotor *motor)
{
    uint8_t params[1] = { 0x52 };
    return send_command(motor, ZDT_CMD_CLEAR_STALL, params, 1);
}

bool ZDT_SetVelocity(ZdtMotor *motor, int16_t rpm, uint8_t acceleration)
{
    if (!motor) return false;

    uint8_t dir;
    uint16_t speed;

    if (rpm >= 0) {
        dir = ZDT_DIR_CW;
        speed = (uint16_t)rpm;
    } else {
        dir = ZDT_DIR_CCW;
        speed = (uint16_t)(-rpm);
    }
    if (speed > 5000) speed = 5000;

    uint8_t params[5];
    params[0] = dir;
    params[1] = (uint8_t)(speed >> 8);
    params[2] = (uint8_t)(speed & 0xFF);
    params[3] = acceleration;
    params[4] = 0x00;

    return send_command(motor, ZDT_CMD_VELOCITY, params, 5);
}

bool ZDT_MoveAbsolute(ZdtMotor *motor, int32_t target_pos,
                       uint16_t rpm, uint8_t acceleration)
{
    if (!motor) return false;

    motor->target_position = target_pos;

    uint8_t dir;
    uint32_t magnitude;

    if (target_pos >= 0) {
        dir = ZDT_DIR_CW;
        magnitude = (uint32_t)target_pos;
    } else {
        dir = ZDT_DIR_CCW;
        magnitude = (uint32_t)(-(int64_t)target_pos); /* Safe for INT32_MIN */
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

    return send_command(motor, ZDT_CMD_POSITION, params, 10);
}

bool ZDT_Home(ZdtMotor *motor, ZdtHomeMode mode)
{
    uint8_t params[2] = { (uint8_t)mode, 0x00 };
    return send_command(motor, ZDT_CMD_HOME, params, 2);
}

bool ZDT_SetZero(ZdtMotor *motor)
{
    uint8_t params[1] = { 0x6D };
    return send_command(motor, ZDT_CMD_SET_ZERO, params, 1);
}

bool ZDT_RequestPosition(ZdtMotor *motor)
{
    return send_command(motor, ZDT_CMD_READ_POSITION, NULL, 0);
}

bool ZDT_RequestSpeed(ZdtMotor *motor)
{
    return send_command(motor, ZDT_CMD_READ_SPEED, NULL, 0);
}

bool ZDT_RequestPositionError(ZdtMotor *motor)
{
    return send_command(motor, ZDT_CMD_READ_POS_ERR, NULL, 0);
}

bool ZDT_RequestStatus(ZdtMotor *motor)
{
    return send_command(motor, ZDT_CMD_READ_STATUS, NULL, 0);
}

bool ZDT_RequestHomeStatus(ZdtMotor *motor)
{
    return send_command(motor, ZDT_CMD_READ_HOME_ST, NULL, 0);
}

/* ========================================================================
 * Unit conversion: encoder counts → command pulses
 *
 * 0x36/0x37 return encoder counts (65536 per revolution).
 * 0xFD uses command pulses (3200 per rev at 16 microsteps).
 * ======================================================================== */
#define ZDT_ENCODER_COUNTS_PER_REV   65536L
#define ZDT_COMMAND_PULSES_PER_REV    3200L

static int32_t encoder_to_pulse(int32_t encoder_count)
{
    int64_t value = (int64_t)encoder_count * ZDT_COMMAND_PULSES_PER_REV;
    if (value >= 0)
        value += ZDT_ENCODER_COUNTS_PER_REV / 2;
    else
        value -= ZDT_ENCODER_COUNTS_PER_REV / 2;
    return (int32_t)(value / ZDT_ENCODER_COUNTS_PER_REV);
}

/* ========================================================================
 * Response parsing
 * ======================================================================== */

static void parse_position_response(ZdtMotor *motor, const uint8_t *data)
{
    /* data[0] = dir, data[1..4] = 4-byte encoder count */
    uint8_t dir = data[0];
    uint32_t magnitude = ((uint32_t)data[1] << 24) |
                         ((uint32_t)data[2] << 16) |
                         ((uint32_t)data[3] << 8)  |
                         (uint32_t)data[4];

    int32_t encoder_count = (dir == ZDT_DIR_CCW)
                          ? -(int32_t)magnitude
                          :  (int32_t)magnitude;

    motor->position = encoder_to_pulse(encoder_count);
}

static void parse_velocity_response(ZdtMotor *motor, const uint8_t *data)
{
    /* data[0] = dir, data[1..2] = 2-byte speed magnitude */
    uint8_t dir = data[0];
    uint16_t speed_raw = ((uint16_t)data[1] << 8) | data[2];

    if (dir == ZDT_DIR_CCW) {
        motor->speed = -(int16_t)speed_raw;
    } else {
        motor->speed = (int16_t)speed_raw;
    }
}

static void parse_position_error_response(ZdtMotor *motor, const uint8_t *data)
{
    uint8_t dir = data[0];
    uint32_t magnitude = ((uint32_t)data[1] << 24) |
                         ((uint32_t)data[2] << 16) |
                         ((uint32_t)data[3] << 8)  |
                         (uint32_t)data[4];

    int32_t encoder_count = (dir == ZDT_DIR_CCW)
                          ? -(int32_t)magnitude
                          :  (int32_t)magnitude;

    motor->position_error = encoder_to_pulse(encoder_count);
}

static void parse_status_response(ZdtMotor *motor, const uint8_t *data)
{
    motor->status_flags = data[0];
}

static void parse_home_status_response(ZdtMotor *motor, const uint8_t *data)
{
    motor->home_status = data[0];
}

void ZDT_Poll(ZdtMotor *motor)
{
    if (!motor) return;

    /* ---- 1. Process complete frame ---- */
    if (motor->rx.frame_ready) {
        uint8_t func = motor->rx.func_code;
        uint8_t *data = motor->rx.data;
        uint8_t dlen = motor->rx.data_idx;

        /* Verify address matches */
        if (motor->rx.address == motor->cfg.address) {

            /* Check if this response matches the pending command */
            bool is_response_to_pending =
                (motor->tx.result == ZDT_TX_PENDING) &&
                (func == motor->tx.expected_cmd);

            switch (func) {
            /* ACK responses to control commands */
            case ZDT_CMD_ENABLE:
            case ZDT_CMD_STOP:
            case ZDT_CMD_VELOCITY:
            case ZDT_CMD_POSITION:
            case ZDT_CMD_HOME:
            case ZDT_CMD_SET_ZERO:
            case ZDT_CMD_CLEAR_STALL:
                if (is_response_to_pending && dlen >= 1) {
                    motor->tx.response_code = data[0];
                    if (data[0] == ZDT_RESP_ACK) {
                        motor->tx.result = ZDT_TX_ACK;
                    } else {
                        /* 0xE2 or 0xEE = NACK */
                        motor->tx.result = ZDT_TX_NACK;
                    }
                }
                break;

            /* Query responses */
            case ZDT_CMD_READ_POSITION:
                if (dlen >= 5) parse_position_response(motor, data);
                if (is_response_to_pending) motor->tx.result = ZDT_TX_ACK;
                break;

            case ZDT_CMD_READ_SPEED:
                if (dlen >= 3) parse_velocity_response(motor, data);
                if (is_response_to_pending) motor->tx.result = ZDT_TX_ACK;
                break;

            case ZDT_CMD_READ_POS_ERR:
                if (dlen >= 5) parse_position_error_response(motor, data);
                if (is_response_to_pending) motor->tx.result = ZDT_TX_ACK;
                break;

            case ZDT_CMD_READ_STATUS:
                if (dlen >= 1) parse_status_response(motor, data);
                if (is_response_to_pending) motor->tx.result = ZDT_TX_ACK;
                break;

            case ZDT_CMD_READ_HOME_ST:
                if (dlen >= 1) parse_home_status_response(motor, data);
                if (is_response_to_pending) motor->tx.result = ZDT_TX_ACK;
                break;

            default:
                /* Unknown func code — ignore */
                break;
            }
        }

        motor->rx.frame_ready = false;
    }

    /* ---- 2. Check transaction timeout ---- */
    if (motor->tx.result == ZDT_TX_PENDING) {
        if (motor->cfg.get_tick_ms) {
            uint32_t now = motor->cfg.get_tick_ms();
            if ((int32_t)(now - motor->tx.deadline_ms) > 0) {
                motor->tx.result = ZDT_TX_TIMEOUT;
            }
        }
    }
}

/* ========================================================================
 * Software limit check
 * ======================================================================== */

ZdtResult ZDT_CheckLimits(const ZdtMotor *motor, int32_t target)
{
    if (!motor) return ZDT_ERROR_INVALID_PARAM;
    if (target < motor->cfg.limit_min || target > motor->cfg.limit_max) {
        return ZDT_ERROR_SOFT_LIMIT;
    }
    return ZDT_OK;
}

bool ZDT_SaveZero(ZdtMotor *motor)
{
    uint8_t params[2] = { 0x88, 0x01 };  /* 0x01 = save to Flash */
    return send_command(motor, 0x93, params, 2);
}
