/**
 * gray_sensor.c — 8路灰度传感器 串行移位驱动 (MSPM0 DriverLib)
 *
 * 协议: CLK 发脉冲 → 读 DAT → 移位 → 8 次后得到完整字节
 */

#include "gray_sensor.h"

uint8_t GraySensor_Read(void)
{
    uint8_t data = 0;
    uint8_t i;

    DL_GPIO_clearPins(GRAY_SENSOR_PORT, GRAY_SENSOR_GRAY_CLK_PIN);
    for (volatile uint8_t d = 0; d < 20; d++);

    for (i = 0; i < 8; i++) {
        /* 上升沿 → 传感器更新 DAT → 读 */
        DL_GPIO_setPins(GRAY_SENSOR_PORT, GRAY_SENSOR_GRAY_CLK_PIN);
        for (volatile uint8_t d = 0; d < 20; d++);

        if (!DL_GPIO_readPins(GRAY_SENSOR_PORT, GRAY_SENSOR_GRAY_DAT_PIN))
            data |= (1 << i);           /* DAT=低=黑线→bit置1 */

        DL_GPIO_clearPins(GRAY_SENSOR_PORT, GRAY_SENSOR_GRAY_CLK_PIN);
        for (volatile uint8_t d = 0; d < 10; d++);
    }

    return data;
}


int16_t GraySensor_GetPosition(uint8_t raw)
{
    int32_t sum_weight = 0, sum_count = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (raw & (1 << i)) {           /* bit=1 → 黑线 */
            sum_weight += i * 100;
            sum_count++;
        }
    }

    if (sum_count == 0) return -1;
    return (int16_t)(sum_weight / sum_count);
}
