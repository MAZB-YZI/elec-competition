/**
 * gray_sensor.h — 感为科技 8路灰度传感器 (I2C)
 *
 * 依赖: SysConfig 生成的 ti_msp_dl_config.h
 * I2C 总线: I2C_GRAY (I2C1, PA30=SDA, PA29=SCL)
 * 7-bit 从机地址: 0x4C
 * 8-bit 写地址: 0x98, 读地址: 0x99
 * 数据格式: 1 字节, bit0~bit7 对应 8 路灰度 (0=黑线, 1=白底)
 */

#ifndef __GRAY_SENSOR_H__
#define __GRAY_SENSOR_H__

#include "ti_msp_dl_config.h"

#define GRAY_SENSOR_ADDR  0x4C

/**
 * @brief 读取 8 路灰度值
 * @return 8-bit 数据, 每个 bit 对应一路传感器 (1=白, 0=黑)
 *         可直接用于巡线判断: 哪一位为 0 即是黑线位置
 */
uint8_t GraySensor_Read(void);

/**
 * @brief 获取黑线中心位置 (加权平均)
 * @return 0~700 的位置值 (0=最左, 350=居中, 700=最右)
 *         返回 -1 表示全线丢失
 */
int16_t GraySensor_GetPosition(void);

#endif /* __GRAY_SENSOR_H__ */
