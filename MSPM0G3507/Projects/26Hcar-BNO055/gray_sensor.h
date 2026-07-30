/**
 * gray_sensor.h — 8路灰度传感器 (串行移位输出)
 *
 * 依赖: SysConfig 生成的 ti_msp_dl_config.h
 * 引脚: PB6(CLK 输出), PB7(DAT 输入)
 * 协议: CLK 发 8 个脉冲, DAT 线串行读出 8 路灰度值
 */

#ifndef __GRAY_SENSOR_H__
#define __GRAY_SENSOR_H__

#include "ti_msp_dl_config.h"

uint8_t GraySensor_Read(void);
int16_t GraySensor_GetPosition(uint8_t raw);       /* 用已读 raw 直接算, 不重复读 */

#endif
