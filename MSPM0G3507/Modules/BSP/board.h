#ifndef _BOARD_H_
#define _BOARD_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- 基础类型定义 ---- */
typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;
typedef volatile uint32_t vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

#define ABS(a) (a>0 ? a:(-a))

/* ---- SysTick 常量 ---- */
#define SysTickMAX_COUNT 0xFFFFFF
#define SysTickFre       80000000

/* ---- 模块头文件 ---- */
#include "oled.h"
#include "led.h"
#include "key.h"
#include "delay.h"

#endif /* _BOARD_H_ */
