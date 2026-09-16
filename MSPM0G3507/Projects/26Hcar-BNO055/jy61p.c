/**
 * jy61p.c — JY61P UART IMU driver
 *
 * UART0: PA0=TX, PA1=RX, 9600 baud.
 * WIT frames are 11 bytes: 0x55, type, payload[8], checksum.
 */

#include "jy61p.h"
#include "ti_msp_dl_config.h"

#define WIT_FRAME_HEADER       0x55U
#define WIT_FRAME_ACCEL        0x51U
#define WIT_FRAME_ANGLE        0x53U
#define WIT_FRAME_LEN          11U

#define ANGLE_SCALE_DEG        (180.0f / 32768.0f)
#define ACCEL_SCALE_G          (16.0f / 32768.0f)
#define STANDARD_GRAVITY_MPS2  9.80665f

typedef enum {
    STATE_IDLE = 0,
    STATE_HEADER,
    STATE_DATA
} RxState_t;

static volatile RxState_t rx_state;
static volatile uint8_t rx_buf[WIT_FRAME_LEN];
static volatile uint8_t rx_idx;

static volatile int16_t raw_roll;
static volatile int16_t raw_pitch;
static volatile int16_t raw_yaw;
static volatile int16_t raw_angle_temp;

static volatile int16_t raw_accel_x;
static volatile int16_t raw_accel_y;
static volatile int16_t raw_accel_z;
static volatile int16_t raw_accel_temp;

static volatile uint32_t angle_frame_count;
static volatile uint32_t accel_frame_count;
static volatile uint32_t last_valid_frame_tick;
static volatile uint32_t last_accel_frame_tick;  /* 最近一次 0x51 帧的 tick */
static volatile uint32_t tick_5ms;
static float yaw_offset;

static int16_t le_i16(uint8_t lo, uint8_t hi)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

void JY61P_Init(void)
{
    rx_state = STATE_IDLE;
    rx_idx = 0U;
    raw_roll = 0;
    raw_pitch = 0;
    raw_yaw = 0;
    raw_angle_temp = 0;
    raw_accel_x = 0;
    raw_accel_y = 0;
    raw_accel_z = 0;
    raw_accel_temp = 0;
    angle_frame_count = 0U;
    accel_frame_count = 0U;
    last_valid_frame_tick = 0U;
    tick_5ms = 0U;
    yaw_offset = 0.0f;

    NVIC_EnableIRQ(JY61P_UART_INST_INT_IRQN);
    DL_UART_enableInterrupt(JY61P_UART_INST, DL_UART_INTERRUPT_RX);
}

static void process_frame(void)
{
    uint8_t sum = 0U;

    for (uint8_t i = 0U; i < WIT_FRAME_LEN - 1U; i++) {
        sum = (uint8_t)(sum + rx_buf[i]);
    }
    if (sum != rx_buf[WIT_FRAME_LEN - 1U]) return;

    if (rx_buf[1] == WIT_FRAME_ANGLE) {
        raw_roll = le_i16(rx_buf[2], rx_buf[3]);
        raw_pitch = le_i16(rx_buf[4], rx_buf[5]);
        raw_yaw = le_i16(rx_buf[6], rx_buf[7]);
        raw_angle_temp = le_i16(rx_buf[8], rx_buf[9]);
        angle_frame_count++;
        last_valid_frame_tick = tick_5ms;
    } else if (rx_buf[1] == WIT_FRAME_ACCEL) {
        raw_accel_x = le_i16(rx_buf[2], rx_buf[3]);
        raw_accel_y = le_i16(rx_buf[4], rx_buf[5]);
        raw_accel_z = le_i16(rx_buf[6], rx_buf[7]);
        raw_accel_temp = le_i16(rx_buf[8], rx_buf[9]);
        accel_frame_count++;
        last_valid_frame_tick = tick_5ms;
        last_accel_frame_tick = tick_5ms;
    }
}

void JY61P_UART_IRQHandler(void)
{
    if (DL_UART_getEnabledInterruptStatus(JY61P_UART_INST,
            DL_UART_INTERRUPT_RX) != DL_UART_INTERRUPT_RX) {
        return;
    }

    uint8_t byte = DL_UART_receiveData(JY61P_UART_INST);

    switch (rx_state) {
    case STATE_IDLE:
        if (byte == WIT_FRAME_HEADER) {
            rx_buf[0] = byte;
            rx_idx = 1U;
            rx_state = STATE_HEADER;
        }
        break;

    case STATE_HEADER:
        if (byte == WIT_FRAME_ACCEL || byte == WIT_FRAME_ANGLE) {
            rx_buf[1] = byte;
            rx_idx = 2U;
            rx_state = STATE_DATA;
        } else if (byte == WIT_FRAME_HEADER) {
            rx_buf[0] = byte;
            rx_idx = 1U;
        } else {
            rx_state = STATE_IDLE;
            rx_idx = 0U;
        }
        break;

    case STATE_DATA:
        rx_buf[rx_idx++] = byte;
        if (rx_idx >= WIT_FRAME_LEN) {
            process_frame();
            rx_state = STATE_IDLE;
            rx_idx = 0U;
        }
        break;

    default:
        rx_state = STATE_IDLE;
        rx_idx = 0U;
        break;
    }

    DL_UART_clearInterruptStatus(JY61P_UART_INST, DL_UART_INTERRUPT_RX);
}

float JY61P_GetYaw(void)
{
    float yaw = (float)raw_yaw * ANGLE_SCALE_DEG - yaw_offset;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

int16_t JY61P_GetYawRaw(void) { return raw_yaw; }
float JY61P_GetRoll(void) { return (float)raw_roll * ANGLE_SCALE_DEG; }
float JY61P_GetPitch(void) { return (float)raw_pitch * ANGLE_SCALE_DEG; }

void JY61P_ZeroYaw(void)
{
    yaw_offset = (float)raw_yaw * ANGLE_SCALE_DEG;
}

int16_t JY61P_GetAccelXRaw(void) { return raw_accel_x; }
int16_t JY61P_GetAccelYRaw(void) { return raw_accel_y; }
int16_t JY61P_GetAccelZRaw(void) { return raw_accel_z; }

float JY61P_GetAccelXG(void) { return (float)raw_accel_x * ACCEL_SCALE_G; }
float JY61P_GetAccelYG(void) { return (float)raw_accel_y * ACCEL_SCALE_G; }
float JY61P_GetAccelZG(void) { return (float)raw_accel_z * ACCEL_SCALE_G; }
float JY61P_GetAccelXMps2(void)
{
    return JY61P_GetAccelXG() * STANDARD_GRAVITY_MPS2;
}
float JY61P_GetAccelYMps2(void)
{
    return JY61P_GetAccelYG() * STANDARD_GRAVITY_MPS2;
}
float JY61P_GetAccelZMps2(void)
{
    return JY61P_GetAccelZG() * STANDARD_GRAVITY_MPS2;
}

uint32_t JY61P_GetFrameCount(void) { return angle_frame_count; }
uint32_t JY61P_GetAccelFrameCount(void) { return accel_frame_count; }
bool JY61P_HasAcceleration(void) { return accel_frame_count > 0U; }

/* 最近 200ms 内收到过 0x51 加速度帧 */
#define ACCEL_FRESH_TICKS  40U  /* 40 × 5ms = 200ms */
bool JY61P_IsAccelFresh(void)
{
    return accel_frame_count > 0U &&
           (tick_5ms - last_accel_frame_tick) < ACCEL_FRESH_TICKS;
}

bool JY61P_IsOnline(void)
{
    return (angle_frame_count > 0U || accel_frame_count > 0U) &&
           ((tick_5ms - last_valid_frame_tick) < 100U);
}

void JY61P_UpdateTick(void)
{
    tick_5ms++;
}
