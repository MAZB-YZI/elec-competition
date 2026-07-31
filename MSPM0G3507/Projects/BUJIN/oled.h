#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 适配 syscfg: 仓库用 OLED_INST → 映射到系统生成的 I2C_BUS_INST */
#define OLED_INST I2C_BUS_INST

/* ========== 基本参数 ========== */
#define OLED_WIDTH        128
#define OLED_HEIGHT       64
#define OLED_PAGES        8
#define OLED_ADDR         0x3C    // 7位I2C地址

/* ========== 命令/数据模式 ========== */
#define OLED_CMD          0
#define OLED_DATA         1

/* ========== 显存缓冲区 ========== */
extern uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGES];

/* ========== 基础操作 ========== */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearBuffer(void);
void OLED_Refresh(void);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);

/* ========== 显示控制 ========== */
void OLED_ColorTurn(uint8_t i);       // 0:正常 1:反色
void OLED_DisplayTurn(uint8_t i);     // 0:正常 1:旋转180°

/* ========== 绘图功能 ========== */
void OLED_DrawPoint(uint8_t x, uint8_t y);
void OLED_ClearPoint(uint8_t x, uint8_t y);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);

/* ========== 显示功能 ========== */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1);
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1);
void OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[]);

/* ========== 工具函数 ========== */
void OLED_WR_BP(uint8_t x, uint8_t y);

#endif
