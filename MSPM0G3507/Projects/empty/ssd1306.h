#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* SSD1306 I2C地址 (7位) */
/* 默认: Pin1接GND, Pin2接VCC -> SA0=0 -> 0x3C */
#define SSD1306_ADDR      0x3C

/* 如果0R电阻改为 Pin1接VCC, Pin2接GND -> SA0=1 -> 0x3D */
/* #define SSD1306_ADDR    0x3D */

#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT    64
#define SSD1306_PAGES     8

void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawString(uint8_t page, uint8_t col, const char *str);

#endif
