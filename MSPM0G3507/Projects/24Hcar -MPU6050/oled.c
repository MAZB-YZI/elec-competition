#include "oled.h"
#include "oledfont.h"
#include "delay.h"
#include <string.h>

uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGES];
static uint8_t OLED_Dirty;
static uint8_t OLED_First = 1;

#define OLED_SDA_PORT GPIOA
#define OLED_SDA_PIN  DL_GPIO_PIN_28
#define OLED_SDA_IOMUX IOMUX_PINCM3
#define OLED_SCL_PORT GPIOA
#define OLED_SCL_PIN  DL_GPIO_PIN_31
#define OLED_SCL_IOMUX IOMUX_PINCM6
#define OLED_I2C_DELAY_US 2U

static uint8_t OLED_SoftInited;

static void OLED_SDA_Release(void)
{
    DL_GPIO_initDigitalInputFeatures(OLED_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

static void OLED_SCL_Release(void)
{
    DL_GPIO_initDigitalInputFeatures(OLED_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
}

static void OLED_SDA_Low(void)
{
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN);
    DL_GPIO_enableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

static void OLED_SCL_Low(void)
{
    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN);
    DL_GPIO_enableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
}

static void OLED_SoftI2C_Init(void)
{
    if (OLED_SoftInited) return;
    OLED_SDA_Release();
    OLED_SCL_Release();
    delay_us(20U);
    OLED_SoftInited = 1U;
}

static void OLED_I2C_Start(void)
{
    OLED_SDA_Release();
    OLED_SCL_Release();
    delay_us(OLED_I2C_DELAY_US);
    OLED_SDA_Low();
    delay_us(OLED_I2C_DELAY_US);
    OLED_SCL_Low();
}

static void OLED_I2C_Stop(void)
{
    OLED_SDA_Low();
    delay_us(OLED_I2C_DELAY_US);
    OLED_SCL_Release();
    delay_us(OLED_I2C_DELAY_US);
    OLED_SDA_Release();
    delay_us(OLED_I2C_DELAY_US);
}

static void OLED_I2C_WriteByte(uint8_t data)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        OLED_SCL_Low();
        if (data & 0x80U) OLED_SDA_Release();
        else OLED_SDA_Low();
        delay_us(OLED_I2C_DELAY_US);
        OLED_SCL_Release();
        delay_us(OLED_I2C_DELAY_US);
        data <<= 1;
    }
    OLED_SCL_Low();
    OLED_SDA_Release();
    delay_us(OLED_I2C_DELAY_US);
    OLED_SCL_Release();
    delay_us(OLED_I2C_DELAY_US);
    OLED_SCL_Low();
}

static void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    OLED_SoftI2C_Init();
    OLED_I2C_Start();
    OLED_I2C_WriteByte((uint8_t)(OLED_ADDR << 1));
    OLED_I2C_WriteByte(mode ? 0x40U : 0x00U);
    OLED_I2C_WriteByte(dat);
    OLED_I2C_Stop();
}
void OLED_ColorTurn(uint8_t i)
{
    if (i == 0) OLED_WR_Byte(0xA6, OLED_CMD);  // 姝ｅ父鏄剧ず
    if (i == 1) OLED_WR_Byte(0xA7, OLED_CMD);  // 鍙嶈壊鏄剧ず
}

void OLED_DisplayTurn(uint8_t i)
{
    if (i == 0) {
        OLED_WR_Byte(0xC8, OLED_CMD);  // 姝ｅ父鏄剧ず
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1) {
        OLED_WR_Byte(0xC0, OLED_CMD);  // 鏃嬭浆180掳
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

/* ========== 鍩虹鎿嶄綔 ========== */
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);  // 鐢佃嵎娉典娇鑳?
    OLED_WR_Byte(0x14, OLED_CMD);  // 寮€鍚數鑽锋车
    OLED_WR_Byte(0xAF, OLED_CMD);  // 鐐逛寒灞忓箷
}

void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);  // 鐢佃嵎娉典娇鑳?
    OLED_WR_Byte(0x10, OLED_CMD);  // 鍏抽棴鐢佃嵎娉?
    OLED_WR_Byte(0xAE, OLED_CMD);  // 鍏抽棴灞忓箷
}

void OLED_Refresh(void)
{
    uint8_t mask = OLED_First ? 0xFF : OLED_Dirty;
    OLED_First = 0;
    OLED_Dirty = 0;

    for (uint8_t p = 0; p < OLED_PAGES; p++) {
        if (!(mask & (1 << p))) continue;
        OLED_WR_Byte(0xB0 + p, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (uint8_t n = 0; n < OLED_WIDTH; n++)
            OLED_WR_Byte(OLED_GRAM[n][p], OLED_DATA);
    }
}

void OLED_Clear(void)
{
    memset(OLED_GRAM, 0x00, sizeof(OLED_GRAM));
    OLED_Dirty = 0xFF;
    OLED_First = 1;
    OLED_Refresh();
}

/* ========== 缁樺浘鍔熻兘 ========== */
void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint8_t pg = y / 8;
    OLED_GRAM[x][pg] |= (1 << (y % 8));
    OLED_Dirty |= (1 << pg);
}

void OLED_ClearPoint(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint8_t pg = y / 8;
    OLED_GRAM[x][pg] &= ~(1 << (y % 8));
    OLED_Dirty |= (1 << pg);
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    if (x1 >= OLED_WIDTH || x2 >= OLED_WIDTH || y1 >= OLED_HEIGHT || y2 >= OLED_HEIGHT) return;
    if (x1 > x2 || y1 > y2) return;

    if (x1 == x2) {  // 绔栫嚎
        for (uint8_t i = 0; i < (y2 - y1); i++)
            OLED_DrawPoint(x1, y1 + i);
    } else if (y1 == y2) {  // 妯嚎
        for (uint8_t i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1);
    } else {  // 鏂滅嚎
        uint8_t k1 = y2 - y1;
        uint8_t k2 = x2 - x1;
        uint8_t k = k1 * 10 / k2;
        for (uint8_t i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1 + i * k / 10);
    }
}

void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a = 0, b = r, num;
    while (2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b);
        OLED_DrawPoint(x - a, y - b);
        OLED_DrawPoint(x - a, y + b);
        OLED_DrawPoint(x + a, y + b);
        OLED_DrawPoint(x + b, y + a);
        OLED_DrawPoint(x + b, y - a);
        OLED_DrawPoint(x - b, y - a);
        OLED_DrawPoint(x - b, y + a);

        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0) { b--; a--; }
    }
}

/* ========== 鏄剧ず鍔熻兘 ========== */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t y0 = y;
    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';

    for (i = 0; i < size2; i++) {
        if (size1 == 12) temp = asc2_1206[chr1][i];
        else if (size1 == 16) temp = asc2_1608[chr1][i];
        else if (size1 == 24) temp = asc2_2412[chr1][i];
        else return;

        for (m = 0; m < 8; m++) {
            if (temp & 0x80) OLED_DrawPoint(x, y);
            else OLED_ClearPoint(x, y);
            temp <<= 1;
            y++;
            if ((y - y0) == size1) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1)
{
    while ((*chr >= ' ') && (*chr <= '~')) {
        OLED_ShowChar(x, y, *chr, size1);
        x += size1 / 2;
        if (x > OLED_WIDTH - size1) {  // 鎹㈣
            x = 0;
            y += size1;
        }
        chr++;
    }
}

static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1)
{
    uint8_t t, temp;
    for (t = 0; t < len; t++) {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + (size1 / 2) * t, y, temp + '0', size1);
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1)
{
    uint8_t i, m, n = 0, temp, chr1;
    uint8_t x0 = x, y0 = y;
    uint8_t size3 = size1 / 8;

    while (size3--) {
        chr1 = num * size1 / 8 + n;
        n++;
        for (i = 0; i < size1; i++) {
            if (size1 == 16) temp = Hzk1[chr1][i];
            else if (size1 == 24) temp = Hzk2[chr1][i];
            else if (size1 == 32) temp = Hzk3[chr1][i];
            else if (size1 == 64) temp = Hzk4[chr1][i];
            else return;

            for (m = 0; m < 8; m++) {
                if (temp & 0x01) OLED_DrawPoint(x, y);
                else OLED_ClearPoint(x, y);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == size1) { x = x0; y0 = y0 + 8; }
            y = y0;
        }
    }
}

void OLED_WR_BP(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xB0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xF0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0F) | 0x01, OLED_CMD);
}

void OLED_ShowPicture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[])
{
    uint32_t j = 0;
    uint8_t x, y;

    if (y0 % 8 == 0) y = 0;
    else y = y0 + 1;

    for (y = y0; y < y1; y++) {
        OLED_WR_BP(x0, y);
        for (x = x0; x < x1; x++) {
            OLED_WR_Byte(BMP[j], OLED_DATA);
            j++;
        }
    }
}

/* ========== 鍒濆鍖?========== */
void OLED_Init(void)
{
    delay_ms(100);  // 绛夊緟涓婄數澶嶄綅

    OLED_WR_Byte(0xAE, OLED_CMD);  // 鍏抽棴鏄剧ず
    OLED_WR_Byte(0xD5, OLED_CMD);  // 璁剧疆鏃堕挓鍒嗛
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);  // 璁剧疆澶嶇敤鐜?
    OLED_WR_Byte(0x3F, OLED_CMD);  // 1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD);  // 璁剧疆鏄剧ず鍋忕Щ
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);  // 璁剧疆璧峰琛?
    OLED_WR_Byte(0x8D, OLED_CMD);  // 鐢佃嵎娉典娇鑳?
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);  // 璁剧疆瀵诲潃妯″紡
    OLED_WR_Byte(0x02, OLED_CMD);  // Page addressing
    OLED_WR_Byte(0xA1, OLED_CMD);  // Segment remap
    OLED_WR_Byte(0xC8, OLED_CMD);  // COM鎵弿鏂瑰悜
    OLED_WR_Byte(0xDA, OLED_CMD);  // COM寮曡剼閰嶇疆
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);  // 璁剧疆瀵规瘮搴?
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);  // 棰勫厖鐢靛懆鏈?
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);  // VCOMH鐢靛帇
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);  // 鎭㈠鏄剧ず鍐呭
    OLED_WR_Byte(0xA6, OLED_CMD);  // 姝ｅ父鏄剧ず

    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD);  // 寮€鍚樉绀?
}
