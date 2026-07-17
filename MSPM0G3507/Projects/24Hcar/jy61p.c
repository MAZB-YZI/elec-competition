/**
 * jy61p.c — JY61P 串口陀螺仪驱动
 *
 * 使用 UART0 (PA0=TX, PA1=RX)，波特率 9600
 * WIT 私有协议，角度帧: 0x55 0x53
 */

#include "jy61p.h"
#include "ti_msp_dl_config.h"

/* WIT 协议帧定义 */
#define WIT_FRAME_HEADER    0x55
#define WIT_FRAME_ANGLE     0x53
#define WIT_FRAME_LEN       11

/* 角度换算系数 */
#define ANGLE_SCALE         (180.0f / 32768.0f)   /* 原始值 / 32768 * 180 = 度 */

/* 接收状态机 */
typedef enum {
    STATE_IDLE,      /* 等待帧头 0x55 */
    STATE_HEADER,    /* 收到 0x55，等待帧类型 */
    STATE_DATA       /* 收集数据 */
} RxState_t;

/* 内部变量 */
static volatile RxState_t  rx_state;
static volatile uint8_t    rx_buf[WIT_FRAME_LEN];
static volatile uint8_t    rx_idx;

static volatile int16_t    raw_roll;
static volatile int16_t    raw_pitch;
static volatile int16_t    raw_yaw;
static volatile int16_t    raw_temp;
static volatile uint32_t   frame_count;
static volatile uint32_t   last_frame_ms;

static float    yaw_offset;
static volatile uint32_t   ms_ticks;

/* ================================================================
 *  初始化
 * ================================================================ */
void JY61P_Init(void)
{
    rx_state = STATE_IDLE;
    rx_idx = 0;
    raw_roll = 0;
    raw_pitch = 0;
    raw_yaw = 0;
    raw_temp = 0;
    frame_count = 0;
    last_frame_ms = 0;
    yaw_offset = 0.0f;
    ms_ticks = 0;

    /* UART0 已由 SysConfig 初始化，这里只开启接收中断 */
    NVIC_EnableIRQ(UART0_INT_IRQn);
    DL_UART_enableInterrupt(UART0_INST, DL_UART_INTERRUPT_RX);
}

/* ================================================================
 *  帧解析
 * ================================================================ */
static void process_frame(void)
{
    /* 校验和：前 10 字节之和 = 第 11 字节 */
    uint8_t sum = 0;
    for (uint8_t i = 0; i < WIT_FRAME_LEN - 1; i++) {
        sum += rx_buf[i];
    }
    if (sum != rx_buf[WIT_FRAME_LEN - 1]) {
        return; /* 校验失败 */
    }

    /* 解析角度值（小端序，单位 0.01 度） */
    raw_roll  = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[2]);
    raw_pitch = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[4]);
    raw_yaw   = (int16_t)((uint16_t)rx_buf[7] << 8 | rx_buf[6]);
    raw_temp  = (int16_t)((uint16_t)rx_buf[9] << 8 | rx_buf[8]);

    frame_count++;
    last_frame_ms = ms_ticks;
}

/* ================================================================
 *  UART0 中断处理
 * ================================================================ */
void JY61P_UART_IRQHandler(void)
{
    uint8_t byte;

    if (DL_UART_getEnabledInterruptStatus(UART0_INST,
            DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX) {

        byte = DL_UART_receiveData(UART0_INST);

        switch (rx_state) {
        case STATE_IDLE:
            if (byte == WIT_FRAME_HEADER) {
                rx_buf[0] = byte;
                rx_idx = 1;
                rx_state = STATE_HEADER;
            }
            break;

        case STATE_HEADER:
            if (byte == WIT_FRAME_ANGLE) {
                rx_buf[1] = byte;
                rx_idx = 2;
                rx_state = STATE_DATA;
            } else {
                rx_state = STATE_IDLE;
            }
            break;

        case STATE_DATA:
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= WIT_FRAME_LEN) {
                process_frame();
                rx_state = STATE_IDLE;
            }
            break;
        }

        DL_UART_clearInterruptStatus(UART0_INST, DL_UART_INTERRUPT_RX);
    }
}

/* ================================================================
 *  公共接口
 * ================================================================ */
float JY61P_GetYaw(void)
{
    float yaw = (float)raw_yaw * ANGLE_SCALE - yaw_offset;
    /* 归一化到 -180° ~ +180° */
    while (yaw > 180.0f)  yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

int16_t JY61P_GetYawRaw(void)
{
    return raw_yaw;
}

float JY61P_GetRoll(void)
{
    return (float)raw_roll * ANGLE_SCALE;
}

float JY61P_GetPitch(void)
{
    return (float)raw_pitch * ANGLE_SCALE;
}

uint32_t JY61P_GetFrameCount(void)
{
    return frame_count;
}

void JY61P_ZeroYaw(void)
{
    yaw_offset = (float)raw_yaw * ANGLE_SCALE;
}

bool JY61P_IsOnline(void)
{
    return (ms_ticks - last_frame_ms) < 100U;   /* 100×5ms=500ms */
}

/* ================================================================
 *  1ms 节拍（由定时器中断调用）
 * ================================================================ */
void JY61P_UpdateTick(void)
{
    ms_ticks++;
}
