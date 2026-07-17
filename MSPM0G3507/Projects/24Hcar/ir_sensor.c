/**
 * ir_sensor.c — 红外测距 ADC 采集 (PA15)
 */
#include "ir_sensor.h"

uint16_t IR_Read(void)
{
    DL_ADC12_startConversion(ir_sensor_INST);
    for (volatile uint16_t d = 0; d < 500; d++);
    return (uint16_t)DL_ADC12_getMemResult(ir_sensor_INST, DL_ADC12_MEM_IDX_0);
}
