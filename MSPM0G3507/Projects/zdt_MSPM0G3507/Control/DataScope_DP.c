#include "DataScope_DP.h"
#include "uart_callback.h"
#include <string.h>

volatile BLDC_MotorData_t BLDC_Motor1 = {0};
volatile BLDC_MotorData_t BLDC_Motor2 = {0};

static uint8_t rx_buf[9];
static uint8_t rx_index = 0;
static uint8_t rx_state = 0;

uint8_t Calc_BCC(uint8_t *data, uint8_t len)
{
    uint8_t bcc = 0;
    uint8_t i;

    for (i = 0; i < len; i++) {
        bcc ^= data[i];
    }

    return bcc;
}

void BLDC_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t tx_buf[20];
    uint8_t idx = 0;

    if (len > (sizeof(tx_buf) - 5U)) {
        return;
    }

    tx_buf[idx++] = BLDC_HEADER;
    tx_buf[idx++] = addr;
    tx_buf[idx++] = cmd;

    if ((len > 0U) && (data != NULL)) {
        memcpy(&tx_buf[idx], data, len);
        idx = (uint8_t)(idx + len);
    }

    tx_buf[idx++] = Calc_BCC(tx_buf, idx);
    tx_buf[idx++] = BLDC_TAIL;
    uart1_send_SendArray(tx_buf, idx);
}

void BLDC_Enable(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_ENABLE, NULL, 0);
}

void BLDC_Disable(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_DISABLE, NULL, 0);
}

void BLDC_SetSpeed(uint8_t addr, int16_t rpm)
{
    uint8_t data[2];

    data[0] = (uint8_t)(((uint16_t)rpm >> 8) & 0xFFU);
    data[1] = (uint8_t)((uint16_t)rpm & 0xFFU);
    BLDC_SendCmd(addr, CMD_SPEED, data, 2);
}

void BLDC_SetMode(uint8_t addr, uint16_t mode)
{
    uint8_t data[2];

    data[0] = (uint8_t)((mode >> 8) & 0xFFU);
    data[1] = (uint8_t)(mode & 0xFFU);
    BLDC_SendCmd(addr, CMD_MODE, data, 2);
}

void BLDC_SetMultiAngle(uint8_t addr, int32_t angle_x10)
{
    uint32_t raw = (uint32_t)angle_x10;
    uint8_t data[4];

    data[0] = (uint8_t)((raw >> 24) & 0xFFU);
    data[1] = (uint8_t)((raw >> 16) & 0xFFU);
    data[2] = (uint8_t)((raw >> 8) & 0xFFU);
    data[3] = (uint8_t)(raw & 0xFFU);
    BLDC_SendCmd(addr, CMD_MULTI_POS, data, 4);
}

void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10)
{
    uint8_t data[2];

    if (angle_x10 > 3599U) {
        angle_x10 = 3599U;
    }

    data[0] = (uint8_t)((angle_x10 >> 8) & 0xFFU);
    data[1] = (uint8_t)(angle_x10 & 0xFFU);
    BLDC_SendCmd(addr, CMD_SINGLE_POS, data, 2);
}

void BLDC_ReqFeedback(uint8_t addr, uint8_t type)
{
    uint8_t data[1] = {type};

    BLDC_SendCmd(addr, CMD_FEEDBACK, data, 1);
}

void BLDC_SetAcc(uint8_t addr, uint16_t acc)
{
    uint8_t data[2];

    data[0] = (uint8_t)((acc >> 8) & 0xFFU);
    data[1] = (uint8_t)(acc & 0xFFU);
    BLDC_SendCmd(addr, CMD_ACC, data, 2);
}

void BLDC_SaveParams(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_SAVE, NULL, 0);
}

void BLDC_ClearMultiAngle(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_CLEAR_MULTI, NULL, 0);
}

void BLDC_SetSingleAngleZero(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_SET_ZERO, NULL, 0);
}

void BLDC_FactoryReset(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_FACTORY_RST, NULL, 0);
}

void BLDC_SetAddress(uint8_t addr, uint8_t new_addr)
{
    uint8_t data[1] = {new_addr};

    BLDC_SendCmd(addr, CMD_SET_ADDR, data, 1);
}

static void BLDC_ResetRxParser(void)
{
    rx_index = 0;
    rx_state = 0;
}

void BLDC_ParseRxData(uint8_t rx_byte)
{
    switch (rx_state) {
    case 0:
        if (rx_byte == BLDC_HEADER) {
            rx_buf[0] = rx_byte;
            rx_index = 1;
            rx_state = 1;
        }
        break;

    case 1:
        if ((rx_byte == BLDC_ADDR_1) || (rx_byte == BLDC_ADDR_2)) {
            rx_buf[rx_index++] = rx_byte;
            rx_state = 2;
        } else {
            BLDC_ResetRxParser();
        }
        break;

    case 2:
        if (rx_byte <= FB_VOLTAGE) {
            rx_buf[rx_index++] = rx_byte;
            rx_state = 3;
        } else {
            BLDC_ResetRxParser();
        }
        break;

    case 3:
        rx_buf[rx_index++] = rx_byte;
        if (rx_index >= 7U) {
            rx_state = 4;
        }
        break;

    case 4:
        rx_buf[rx_index++] = rx_byte;
        rx_state = 5;
        break;

    case 5:
        if (rx_byte == BLDC_TAIL) {
            rx_buf[rx_index++] = rx_byte;

            if (Calc_BCC(rx_buf, 7) == rx_buf[7]) {
                uint8_t addr = rx_buf[1];
                uint8_t type = rx_buf[2];
                uint32_t raw = ((uint32_t)rx_buf[3] << 24) |
                               ((uint32_t)rx_buf[4] << 16) |
                               ((uint32_t)rx_buf[5] << 8) |
                               ((uint32_t)rx_buf[6]);
                volatile BLDC_MotorData_t *motor =
                    (addr == BLDC_ADDR_1) ? &BLDC_Motor1 : &BLDC_Motor2;

                switch (type) {
                case FB_SPEED:
                    motor->speed = (int16_t)((uint16_t)raw);
                    break;
                case FB_MULTI_ANGLE:
                    motor->multi_angle = (int32_t)raw;
                    break;
                case FB_SINGLE_ANGLE:
                    motor->single_angle = (uint16_t)raw;
                    break;
                case FB_ACC:
                    motor->acc = (int16_t)((uint16_t)raw);
                    break;
                case FB_VOLTAGE:
                    motor->voltage = (uint16_t)raw;
                    break;
                default:
                    break;
                }

                motor->data_ready = 1;
            }
        }
        BLDC_ResetRxParser();
        break;

    default:
        BLDC_ResetRxParser();
        break;
    }
}