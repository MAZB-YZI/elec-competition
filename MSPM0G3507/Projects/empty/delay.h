#ifndef DELAY_H
#define DELAY_H

#include "ti_msp_dl_config.h"

/**
 * @brief 毫秒延时函数
 * @param ms 延时毫秒数
 */
void delay_ms(uint32_t ms);

/**
 * @brief 微秒延时函数
 * @param us 延时微秒数
 */
void delay_us(uint32_t us);

#endif /* DELAY_H */
