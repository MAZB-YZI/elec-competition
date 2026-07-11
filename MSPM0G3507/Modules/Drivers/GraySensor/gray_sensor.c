/**
 * gray_sensor.c — 感为科技 8路灰度传感器 I2C 驱动 (MSPM0 DriverLib)
 */

#include "gray_sensor.h"

/**
 * @brief 读取 8 路灰度原始值
 */
uint8_t GraySensor_Read(void)
{
    uint8_t data = 0xFF;

    /* 等待总线空闲 */
    while (DL_I2C_getControllerStatus(I2C_GRAY_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY);

    /* 发送寄存器地址 0x00 */
    uint8_t reg = 0x00;
    DL_I2C_fillControllerTXFIFO(I2C_GRAY_INST, &reg, 1);
    DL_I2C_startControllerTransfer(I2C_GRAY_INST,
        GRAY_SENSOR_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_GRAY_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY);

    /* 读取 1 字节 */
    DL_I2C_startControllerTransfer(I2C_GRAY_INST,
        GRAY_SENSOR_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C_GRAY_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY);

    if (!DL_I2C_isControllerRXFIFOEmpty(I2C_GRAY_INST)) {
        data = DL_I2C_receiveControllerData(I2C_GRAY_INST);
    }

    return data;
}

/**
 * @brief 计算黑线位置 (质心法)
 * @return 0~700, -1 表示全线丢失
 */
int16_t GraySensor_GetPosition(void)
{
    uint8_t raw = GraySensor_Read();
    int32_t sum_weight = 0, sum_count = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (!(raw & (1 << i))) {
            sum_weight += i * 100;
            sum_count++;
        }
    }

    if (sum_count == 0) return -1;
    return (int16_t)(sum_weight / sum_count);
}
