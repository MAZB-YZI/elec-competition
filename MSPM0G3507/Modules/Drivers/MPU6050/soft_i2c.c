#include "soft_i2c.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

/*
 * 软件 I2C 驱动
 * SDA = PA0, SCL = PA1
 * 使用 GPIO 模拟 I2C 时序
 * 引脚由 SysConfig 配置为 SOFT_I2C_SDA / SOFT_I2C_SCL
 */

/* 引脚定义 (使用 SysConfig 生成的宏) */
#define SDA_PORT    SOFT_I2C_SDA_PORT
#define SDA_PIN     SOFT_I2C_SDA_SDA_PIN
#define SCL_PORT    SOFT_I2C_SCL_PORT
#define SCL_PIN     SOFT_I2C_SCL_SCL_PIN

/* I2C 时序延时 (约 2.5us @ 80MHz, 400kHz) */
#define I2C_DELAY() delay_us(2)

/* SDA/SCL 控制 */
static void SDA_HIGH(void) { DL_GPIO_setPins(SDA_PORT, SDA_PIN); }
static void SDA_LOW(void)  { DL_GPIO_clearPins(SDA_PORT, SDA_PIN); }
static void SCL_HIGH(void) { DL_GPIO_setPins(SCL_PORT, SCL_PIN); }
static void SCL_LOW(void)  { DL_GPIO_clearPins(SCL_PORT, SCL_PIN); }

static uint8_t SDA_READ(void)
{
    return (DL_GPIO_readPins(SDA_PORT, SDA_PIN) != 0) ? 1 : 0;
}

/* 设置 SDA 为输出 */
static void SDA_OUT(void)
{
    DL_GPIO_enableOutput(SDA_PORT, SDA_PIN);
}

/* 设置 SDA 为输入 (推挽模式，需要切换为输入才能读取实际引脚状态) */
static void SDA_IN(void)
{
    /* 禁用输出，切换为输入模式 */
    DL_GPIO_disableOutput(SDA_PORT, SDA_PIN);
    /* 配置为输入，启用内部上拉 */
    DL_GPIO_initDigitalInputFeatures(SOFT_I2C_SDA_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/* 起始条件 */
static void i2c_start(void)
{
    SDA_OUT();
    SDA_HIGH();
    SCL_HIGH();
    I2C_DELAY();
    SDA_LOW();
    I2C_DELAY();
    SCL_LOW();
    I2C_DELAY();
}

/* 停止条件 */
static void i2c_stop(void)
{
    SDA_OUT();
    SDA_LOW();
    SCL_HIGH();
    I2C_DELAY();
    SDA_HIGH();
    I2C_DELAY();
}

/* 等待 ACK */
static bool i2c_wait_ack(void)
{
    uint8_t timeout = 0;

    SDA_IN();
    SCL_HIGH();
    I2C_DELAY();

    while (SDA_READ()) {
        if (++timeout > 50) {
            i2c_stop();
            return false;
        }
        I2C_DELAY();
    }

    SCL_LOW();
    I2C_DELAY();
    return true;
}

/* 发送 ACK */
static void i2c_ack(void)
{
    SDA_OUT();
    SDA_LOW();
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    SCL_LOW();
    I2C_DELAY();
    SDA_HIGH();
}

/* 发送 NACK */
static void i2c_nack(void)
{
    SDA_OUT();
    SDA_HIGH();
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    SCL_LOW();
    I2C_DELAY();
}

/* 发送一个字节 */
static void i2c_write_byte(uint8_t data)
{
    SDA_OUT();
    SCL_LOW();

    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        data <<= 1;
        I2C_DELAY();
        SCL_HIGH();
        I2C_DELAY();
        SCL_LOW();
        I2C_DELAY();
    }
}

/* 读一个字节 */
static uint8_t i2c_read_byte(bool ack)
{
    uint8_t data = 0;

    SDA_IN();

    for (uint8_t i = 0; i < 8; i++) {
        SCL_LOW();
        I2C_DELAY();
        SCL_HIGH();
        I2C_DELAY();
        data = (data << 1) | SDA_READ();
    }

    if (ack) {
        i2c_ack();
    } else {
        i2c_nack();
    }

    return data;
}

/* 初始化软件 I2C */
void SoftI2C_Init(void)
{
    /* PA0/PA1 已在 SysConfig 中配置为 GPIO 输出 */
    SDA_HIGH();
    SCL_HIGH();
    delay_ms(10U);
}

/* 写寄存器 */
bool SoftI2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t data)
{
    i2c_start();
    i2c_write_byte((addr << 1) | 0);  /* 写地址 */
    if (!i2c_wait_ack()) return false;

    i2c_write_byte(reg);               /* 寄存器地址 */
    if (!i2c_wait_ack()) return false;

    i2c_write_byte(data);              /* 数据 */
    if (!i2c_wait_ack()) return false;

    i2c_stop();
    return true;
}

/* 读寄存器 */
bool SoftI2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data)
{
    i2c_start();
    i2c_write_byte((addr << 1) | 0);  /* 写地址 */
    if (!i2c_wait_ack()) return false;

    i2c_write_byte(reg);               /* 寄存器地址 */
    if (!i2c_wait_ack()) return false;

    i2c_start();                       /* 重复起始 */
    i2c_write_byte((addr << 1) | 1);  /* 读地址 */
    if (!i2c_wait_ack()) return false;

    *data = i2c_read_byte(false);      /* 读数据，发送 NACK */
    i2c_stop();
    return true;
}

/* 读多个字节 */
bool SoftI2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_write_byte((addr << 1) | 0);  /* 写地址 */
    if (!i2c_wait_ack()) return false;

    i2c_write_byte(reg);               /* 寄存器地址 */
    if (!i2c_wait_ack()) return false;

    i2c_start();                       /* 重复起始 */
    i2c_write_byte((addr << 1) | 1);  /* 读地址 */
    if (!i2c_wait_ack()) return false;

    for (uint8_t i = 0; i < len; i++) {
        if (i < len - 1) {
            buf[i] = i2c_read_byte(true);   /* 发送 ACK */
        } else {
            buf[i] = i2c_read_byte(false);  /* 最后一个发送 NACK */
        }
    }

    i2c_stop();
    return true;
}
