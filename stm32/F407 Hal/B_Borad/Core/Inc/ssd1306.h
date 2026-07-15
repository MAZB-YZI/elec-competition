#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "main.h"

/* SSD1306 I2C address: 7-bit 0x3C, shifted left by 1 for STM32 HAL */
#define SSD1306_ADDR      0x78

#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT    64
#define SSD1306_PAGES      8

void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawString(uint8_t page, uint8_t col, const char *str);

#endif
