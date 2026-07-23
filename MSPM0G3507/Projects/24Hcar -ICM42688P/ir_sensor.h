/**
 * ir_sensor.h — 红外测距传感器 (ADC, PA15, 2.5V 内部参考)
 */
#ifndef __IR_SENSOR_H__
#define __IR_SENSOR_H__

#include "ti_msp_dl_config.h"

uint16_t IR_Read(void);   /* 返回 0~4095 */

#endif
