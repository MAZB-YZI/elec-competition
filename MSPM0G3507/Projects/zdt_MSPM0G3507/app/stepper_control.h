/**
 * @file stepper_control.h
 * @brief Stepper motor control application layer
 *
 * State machine:
 *   DISABLED → (enable cmd) → ENABLING → (ACK) → READY
 *   READY → (move cmd) → MOVING → (reached) → READY
 *   READY → (home cmd) → HOMING → (home done) → READY
 *   Any → (fault) → FAULT → (clear) → DISABLED
 *
 * Transaction-driven: state changes on ACK, not on send.
 * Fault entry: STOP first, then DISABLE after STOP completes.
 */
#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include "zdt_emm_v5.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Software position limits (pulse counts) ---- */
#define STEPPER_LIMIT_MIN   (-650)
#define STEPPER_LIMIT_MAX   ( 650)

/* ---- Communication timeout parameters ---- */
#define STEPPER_ACK_TIMEOUT_MS    50   /* Per-command ACK timeout */
#define STEPPER_QUERY_INTERVAL_MS 50   /* Status query period */
#define STEPPER_MAX_TIMEOUTS      3    /* Consecutive timeouts -> FAULT */

/* ---- Position tolerance for "reached" detection ---- */
#define STEPPER_POS_TOLERANCE     5    /* Pulse counts */

/* ---- Error codes ---- */
#define STEPPER_OK                0
#define STEPPER_ERR_FAULT         1
#define STEPPER_ERR_SOFT_LIMIT    2
#define STEPPER_ERR_NOT_HOMED     3
#define STEPPER_ERR_TIMEOUT       4
#define STEPPER_ERR_OFFLINE       5
#define STEPPER_ERR_NACK          6
#define STEPPER_ERR_BUSY          7

/**
 * @brief Initialize stepper control module
 */
void Stepper_Init(void);

/** @brief Send enable command (async — waits for ACK in Poll) */
uint8_t Stepper_Enable(void);

/** @brief Send disable command */
uint8_t Stepper_Disable(void);

/** @brief Emergency stop — STOP + DISABLE, enters DISABLED */
void Stepper_StopNow(void);

/** @brief Move to absolute position (pulses, signed) */
uint8_t Stepper_MoveAbsolute(int32_t target_position,
                              uint16_t rpm, uint8_t acceleration);

/** @brief Move at constant velocity */
uint8_t Stepper_MoveVelocity(int16_t rpm, uint8_t acceleration);

/** @brief Execute homing sequence */
uint8_t Stepper_Home(ZdtHomeMode mode);

/** @brief Set current position as zero */
uint8_t Stepper_SetZero(void);

/** @brief Request position query (non-blocking) */
uint8_t Stepper_QueryPosition(void);

/** @brief Poll state machine — call from main loop */
void Stepper_Poll(void);

/* ---- Status queries ---- */
ZdtState  Stepper_GetState(void);
int32_t   Stepper_GetPosition(void);
int32_t   Stepper_GetPositionError(void);
int16_t   Stepper_GetSpeed(void);
uint32_t  Stepper_GetTimeoutCount(void);
uint8_t   Stepper_GetFaultCode(void);
bool      Stepper_IsFault(void);
bool      Stepper_IsHomed(void);

/** @brief Clear fault — pure software, no protocol commands. FAULT → DISABLED */
uint8_t Stepper_ClearFault(void);

/** @brief Clear stall protection (sends 0x0E command, waits for ACK) */
uint8_t Stepper_ClearStall(void);

/** @brief Query position error (sends 0x37, result in Stepper_GetPositionError) */
uint8_t Stepper_QueryPositionError(void);

/** @brief Save current position as zero to motor Flash (persists across power cycle) */
uint8_t Stepper_SaveZero(void);

/** @brief Get human-readable state name */
const char *Stepper_StateName(ZdtState state);

/** @brief Update software position limits */
void Stepper_SetLimits(int32_t min_pos, int32_t max_pos);

/** @brief Get pointer to underlying ZdtMotor handle */
ZdtMotor *Stepper_GetMotor(void);

#endif /* STEPPER_CONTROL_H */
