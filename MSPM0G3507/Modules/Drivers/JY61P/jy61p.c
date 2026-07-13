#include "jy61p.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

/*
 * JY61P WIT private protocol attitude parser.
 *
 * Expected frame:
 *   0x55 0x53  RollL RollH PitchL PitchH YawL YawH VL VH SUM
 *
 * This module keeps the reusable protocol logic in Modules/ and allows a
 * project-local CCS bridge to include it without hand-editing project metadata.
 */

#ifndef JY61P_UART_INST
#define JY61P_UART_INST UART_JY61P_INST
#endif

typedef enum {
    JY61P_WAIT_HEADER1 = 0,
    JY61P_WAIT_HEADER2 = 1,
    JY61P_RECEIVE_DATA = 2,
} JY61P_ParseState;

static volatile float g_roll_deg = 0.0f;
static volatile float g_pitch_deg = 0.0f;
static volatile float g_yaw_deg = 0.0f;
static volatile uint16_t g_yaw_raw = 0U;
static volatile float g_gyro_z_dps = 0.0f;
static volatile float g_yaw_offset_deg = 0.0f;
static volatile uint32_t g_rx_byte_count = 0U;
static volatile uint32_t g_frame_count = 0U;
static volatile uint32_t g_checksum_error_count = 0U;
static volatile uint32_t g_rx_error_count = 0U;

static volatile JY61P_ParseState g_state = JY61P_WAIT_HEADER1;
static volatile uint8_t g_data_index = 0U;
static uint8_t g_frame_data[9];

static void jy61p_reset_parser(void)
{
    g_state = JY61P_WAIT_HEADER1;
    g_data_index = 0U;
}

static float jy61p_raw_to_deg(uint16_t raw)
{
    float deg = ((float) raw / 32768.0f) * 180.0f;
    if (deg > 180.0f) {
        deg -= 360.0f;
    }
    return deg;
}

static void jy61p_store_frame(const uint8_t *data)
{
    uint8_t sum = 0x55U + 0x53U + data[0] + data[1] + data[2] + data[3] +
                  data[4] + data[5] + data[6] + data[7];
    if (sum != data[8]) {
        ++g_checksum_error_count;
        return;
    }

    uint16_t roll_raw = (uint16_t) (((uint16_t) data[1] << 8) | data[0]);
    uint16_t pitch_raw = (uint16_t) (((uint16_t) data[3] << 8) | data[2]);
    uint16_t yaw_raw = (uint16_t) (((uint16_t) data[5] << 8) | data[4]);

    g_roll_deg = jy61p_raw_to_deg(roll_raw);
    g_pitch_deg = jy61p_raw_to_deg(pitch_raw);
    g_yaw_deg = jy61p_raw_to_deg(yaw_raw);
    g_yaw_raw = yaw_raw;
    g_gyro_z_dps = 0.0f;
    ++g_frame_count;
}

static void jy61p_send_zero_command(void)
{
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0xFFU);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x69U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x88U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0xB5U);
    delay_ms(100U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0xFFU);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x01U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x04U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x00U);
    delay_ms(100U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0xFFU);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x00U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x00U);
    DL_UART_Main_transmitDataBlocking(JY61P_UART_INST, 0x00U);
}

static void jy61p_drain_rx_fifo(void)
{
    while (DL_UART_isRXFIFOEmpty(JY61P_UART_INST) == false) {
        (void) DL_UART_Main_receiveData(JY61P_UART_INST);
        ++g_rx_byte_count;
    }
}

void JY61P_Init(void)
{
    DL_UART_Main_disableInterrupt(JY61P_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_DisableIRQ(UART_JY61P_INST_INT_IRQN);

    jy61p_reset_parser();
    g_roll_deg = 0.0f;
    g_pitch_deg = 0.0f;
    g_yaw_deg = 0.0f;
    g_yaw_raw = 0U;
    g_gyro_z_dps = 0.0f;
    g_yaw_offset_deg = 0.0f;

    /*
     * JY61P 上电后会持续输出数据。
     * 先等模块跑稳，再清一次 RX FIFO，避免复位时卡在半帧或错位帧。
     */
    delay_ms(300U);
    jy61p_drain_rx_fifo();
    jy61p_reset_parser();

    g_rx_byte_count = 0U;
    g_frame_count = 0U;
    g_checksum_error_count = 0U;
    g_rx_error_count = 0U;

    DL_UART_Main_enableInterrupt(JY61P_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_JY61P_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_JY61P_INST_INT_IRQN);
}

void JY61P_ResetYaw(void)
{
    g_yaw_offset_deg = g_yaw_deg;
}

void JY61P_ResetYawTo(float yaw_deg)
{
    g_yaw_offset_deg = g_yaw_deg - yaw_deg;
}

void JY61P_RequestZeroYaw(void)
{
    jy61p_send_zero_command();
    JY61P_ResetYaw();
}

bool JY61P_UpdateStatus(JY61P_Status *status)
{
    if (status == NULL) {
        return false;
    }

    status->roll_deg = g_roll_deg;
    status->pitch_deg = g_pitch_deg;
    status->yaw_deg = g_yaw_deg - g_yaw_offset_deg;
    status->yaw_raw = g_yaw_raw;
    status->gyro_z_dps = g_gyro_z_dps;
    status->rx_byte_count = g_rx_byte_count;
    status->frame_count = g_frame_count;
    status->checksum_error_count = g_checksum_error_count;
    status->rx_error_count = g_rx_error_count;
    return true;
}

float JY61P_GetRoll(void)
{
    return g_roll_deg;
}

float JY61P_GetPitch(void)
{
    return g_pitch_deg;
}

float JY61P_GetYaw(void)
{
    return g_yaw_deg - g_yaw_offset_deg;
}

uint16_t JY61P_GetYawRaw(void)
{
    return g_yaw_raw;
}

float JY61P_GetGyroZ(void)
{
    return g_gyro_z_dps;
}

uint32_t JY61P_GetRxByteCount(void)
{
    return g_rx_byte_count;
}

uint32_t JY61P_GetFrameCount(void)
{
    return g_frame_count;
}

uint32_t JY61P_GetChecksumErrorCount(void)
{
    return g_checksum_error_count;
}

uint32_t JY61P_GetRxErrorCount(void)
{
    return g_rx_error_count;
}

void UART_JY61P_INST_IRQHandler(void)
{
    /* 直接读取数据，不检查中断类型 */
    while (DL_UART_isRXFIFOEmpty(JY61P_UART_INST) == false) {
        uint8_t ch = DL_UART_Main_receiveData(JY61P_UART_INST);
        ++g_rx_byte_count;

        switch (g_state) {
        case JY61P_WAIT_HEADER1:
            if (ch == 0x55U) {
                g_state = JY61P_WAIT_HEADER2;
            }
            break;

        case JY61P_WAIT_HEADER2:
            if (ch == 0x53U) {
                g_state = JY61P_RECEIVE_DATA;
                g_data_index = 0U;
            } else {
                g_state = JY61P_WAIT_HEADER1;
            }
            break;

        case JY61P_RECEIVE_DATA:
            g_frame_data[g_data_index++] = ch;
            if (g_data_index >= sizeof(g_frame_data)) {
                jy61p_store_frame(g_frame_data);
                g_state = JY61P_WAIT_HEADER1;
                g_data_index = 0U;
            }
            break;

        default:
            g_state = JY61P_WAIT_HEADER1;
            g_data_index = 0U;
            break;
        }
    }
}
