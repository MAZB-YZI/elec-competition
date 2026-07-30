/**
 * @file zdt_emm_v5.h
 * @brief ZDT Emm V5 stepper driver protocol layer
 *
 * Frame format: [ADDR] [FUNC] [PARAMS...] [0x6B]
 * All multi-byte values: big-endian.
 * Position protocol: direction + absolute magnitude (not two's complement).
 *
 * Query response format (0x36 position as example):
 *   [ADDR] [0x36] [DIR] [POS3] [POS2] [POS1] [POS0] [0x6B]
 *   5 data bytes after func code: dir(1) + magnitude(4)
 *
 * Design:
 *   - ISR calls ZDT_RxByte() to feed bytes
 *   - Main loop calls ZDT_Poll() to process complete frames
 *   - Transaction result: IDLE/PENDING/ACK/NACK/TIMEOUT
 *   - expected_cmd verification prevents stale responses
 */
#ifndef ZDT_EMM_V5_H
#define ZDT_EMM_V5_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Protocol constants
 * ======================================================================== */

#define ZDT_FRAME_TAIL      0x6B
#define ZDT_DEFAULT_ADDR    0x01
#define ZDT_MAX_DATA_LEN    12
#define ZDT_MAX_FRAME_LEN   16

/* ---- Command function codes ---- */
#define ZDT_CMD_ENABLE          0xF3
#define ZDT_CMD_STOP            0xFE
#define ZDT_CMD_VELOCITY        0xF6
#define ZDT_CMD_POSITION        0xFD
#define ZDT_CMD_HOME            0x9A
#define ZDT_CMD_SET_ZERO        0x0A
#define ZDT_CMD_CLEAR_STALL     0x0E
#define ZDT_CMD_SYNC_TRIGGER    0xFF

/* ---- Query function codes ---- */
#define ZDT_CMD_READ_SPEED      0x35
#define ZDT_CMD_READ_POSITION   0x36
#define ZDT_CMD_READ_POS_ERR    0x37
#define ZDT_CMD_READ_STATUS     0x3A
#define ZDT_CMD_READ_HOME_ST    0x3B

/* ---- ACK response codes (in response byte) ---- */
#define ZDT_RESP_ACK            0x02
#define ZDT_RESP_CMD_ERROR      0xEE
#define ZDT_RESP_COND_NOT_MET   0xE2

/* ---- Direction values ---- */
#define ZDT_DIR_CW              0x00
#define ZDT_DIR_CCW             0x01

/* ---- Position modes ---- */
#define ZDT_POS_MODE_REL_LAST   0x00
#define ZDT_POS_MODE_ABSOLUTE   0x01
#define ZDT_POS_MODE_REL_CURR   0x02

/* ---- Homing modes ---- */
typedef enum {
    ZDT_HOME_SINGLE_NEAREST = 0x00,
    ZDT_HOME_SINGLE_DIR     = 0x01,
    ZDT_HOME_MULTI_COLLIDE  = 0x02,
    ZDT_HOME_MULTI_LIMIT    = 0x03,
} ZdtHomeMode;

/* ---- Parser states ---- */
typedef enum {
    ZDT_PARSE_IDLE,
    ZDT_PARSE_ADDR,
    ZDT_PARSE_FUNC,
    ZDT_PARSE_DATA,
} ZdtParseState;

/* ---- Result codes (function return) ---- */
typedef enum {
    ZDT_OK = 0,
    ZDT_ERROR_TIMEOUT,
    ZDT_ERROR_NACK,
    ZDT_ERROR_SOFT_LIMIT,
    ZDT_ERROR_INVALID_PARAM,
    ZDT_ERROR_BUSY,
} ZdtResult;

/* ---- Transaction result (protocol layer tracks this) ---- */
typedef enum {
    ZDT_TX_IDLE     = 0,  /* No transaction in progress */
    ZDT_TX_PENDING  = 1,  /* Command sent, waiting for response */
    ZDT_TX_ACK      = 2,  /* Received ACK (0x02) */
    ZDT_TX_NACK     = 3,  /* Received NACK (0xE2 or 0xEE) */
    ZDT_TX_TIMEOUT  = 4,  /* Deadline exceeded with no response */
} ZdtTransactionResult;

/* ---- Motor state (application layer) ---- */
typedef enum {
    ZDT_STATE_DISABLED = 0,
    ZDT_STATE_READY,
    ZDT_STATE_MOVING,
    ZDT_STATE_HOMING,
    ZDT_STATE_FAULT,
} ZdtState;

/* ========================================================================
 * Callback types
 * ======================================================================== */

typedef bool (*ZdtWriteFn)(const uint8_t *data, uint16_t len);
typedef uint32_t (*ZdtGetTickFn)(void);

/* ========================================================================
 * Data structures
 * ======================================================================== */

/** @brief Motor configuration (set once at init) */
typedef struct {
    uint8_t       address;
    ZdtWriteFn    write;
    ZdtGetTickFn  get_tick_ms;
    int32_t       limit_min;
    int32_t       limit_max;
} ZdtMotorConfig;

/** @brief Transaction tracking with explicit result */
typedef struct {
    volatile uint32_t          sequence;
    volatile ZdtTransactionResult result;  /* ACK/NACK/TIMEOUT/IDLE */
    uint8_t                    expected_cmd;
    uint8_t                    response_code; /* 0x02/0xE2/0xEE from ACK */
    uint32_t                   deadline_ms;
} ZdtTransaction;

/** @brief RX state machine */
typedef struct {
    volatile ZdtParseState state;
    uint8_t  address;
    uint8_t  func_code;
    uint8_t  data[ZDT_MAX_DATA_LEN];
    uint8_t  data_idx;
    uint8_t  expected_len;
    volatile bool frame_ready;
} ZdtRxState;

/** @brief Motor handle */
typedef struct {
    ZdtMotorConfig cfg;
    ZdtTransaction tx;
    ZdtRxState     rx;
    int32_t  position;         /* Real-time position (pulses, signed) */
    int32_t  position_error;   /* Position following error (pulses) */
    int16_t  speed;            /* Real-time speed (RPM, signed) */
    uint32_t status_flags;     /* Motor status flags */
    uint8_t  home_status;      /* Homing status */
    int32_t  target_position;  /* Last commanded target */
    uint8_t  moving_state;     /* 0=idle, 1=pos, 2=vel, 3=home */
} ZdtMotor;

/* ========================================================================
 * API functions
 * ======================================================================== */

void ZDT_Init(ZdtMotor *motor, const ZdtMotorConfig *cfg);
void ZDT_RxByte(ZdtMotor *motor, uint8_t byte);
void ZDT_Poll(ZdtMotor *motor);

/* ---- Command functions ---- */
bool ZDT_Enable(ZdtMotor *motor, bool enable);
bool ZDT_Stop(ZdtMotor *motor);
bool ZDT_ClearStallProtect(ZdtMotor *motor);
bool ZDT_SetVelocity(ZdtMotor *motor, int16_t rpm, uint8_t acceleration);
bool ZDT_MoveAbsolute(ZdtMotor *motor, int32_t target_pos,
                       uint16_t rpm, uint8_t acceleration);
bool ZDT_Home(ZdtMotor *motor, ZdtHomeMode mode);
bool ZDT_SetZero(ZdtMotor *motor);

/* ---- Query functions ---- */
bool ZDT_RequestPosition(ZdtMotor *motor);
bool ZDT_RequestSpeed(ZdtMotor *motor);
bool ZDT_RequestPositionError(ZdtMotor *motor);
bool ZDT_RequestStatus(ZdtMotor *motor);
bool ZDT_RequestHomeStatus(ZdtMotor *motor);

/* ---- Frame helpers ---- */
uint8_t ZDT_MakeFrame(uint8_t addr, uint8_t func,
                       const uint8_t *params, uint8_t param_len,
                       uint8_t *out_buf);
ZdtResult ZDT_CheckLimits(const ZdtMotor *motor, int32_t target);

/** @brief Save current position as zero to motor Flash (persists across power cycle) */
bool ZDT_SaveZero(ZdtMotor *motor);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_EMM_V5_H */
