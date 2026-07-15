#include "soft_i2c.h"

#include "delay.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

/*
 * Software I2C for the Tianmengxing extension-board MPU6050 connector.
 *
 * Board wiring:
 *   SDA = PA0  (SysConfig GPIO group: SOFT_I2C_SDA)
 *   SCL = PA1  (SysConfig GPIO group: SOFT_I2C_SCL)
 *
 * PA0/PA1 share the same I2C0 alternate-function family as the OLED's
 * PA28/PA31 pins, so the MPU6050 is driven as GPIO bit-banged I2C here.
 *
 * Important: I2C high level means "release the bus", not actively drive high.
 * This implementation drives low with GPIO output enabled and releases high by
 * disabling the output driver, relying on the 3.3 V pull-up.
 */

#define SDA_PORT SOFT_I2C_SDA_PORT
#define SDA_PIN  SOFT_I2C_SDA_SDA_PIN
#define SCL_PORT SOFT_I2C_SCL_PORT
#define SCL_PIN  SOFT_I2C_SCL_SCL_PIN

#define SOFT_I2C_DELAY_US        5U
#define SOFT_I2C_ACK_TIMEOUT     80U
#define SOFT_I2C_SCL_TIMEOUT     80U
#define SOFT_I2C_RECOVERY_CLOCKS 9U

#define I2C_DELAY() delay_us(SOFT_I2C_DELAY_US)

static void sda_input_pullup(void)
{
    DL_GPIO_initDigitalInputFeatures(SOFT_I2C_SDA_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(SDA_PORT, SDA_PIN);
}

static void scl_input_pullup(void)
{
    DL_GPIO_initDigitalInputFeatures(SOFT_I2C_SCL_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(SCL_PORT, SCL_PIN);
}

static void sda_low(void)
{
    DL_GPIO_initDigitalOutput(SOFT_I2C_SDA_SDA_IOMUX);
    DL_GPIO_clearPins(SDA_PORT, SDA_PIN);
    DL_GPIO_enableOutput(SDA_PORT, SDA_PIN);
}

static void scl_low(void)
{
    DL_GPIO_initDigitalOutput(SOFT_I2C_SCL_SCL_IOMUX);
    DL_GPIO_clearPins(SCL_PORT, SCL_PIN);
    DL_GPIO_enableOutput(SCL_PORT, SCL_PIN);
}

static void sda_release(void)
{
    sda_input_pullup();
}

static void scl_release(void)
{
    scl_input_pullup();
}

static bool sda_is_high(void)
{
    return (DL_GPIO_readPins(SDA_PORT, SDA_PIN) & SDA_PIN) != 0U;
}

static bool scl_is_high(void)
{
    return (DL_GPIO_readPins(SCL_PORT, SCL_PIN) & SCL_PIN) != 0U;
}

static bool wait_scl_high(void)
{
    for (uint16_t t = 0U; t < SOFT_I2C_SCL_TIMEOUT; ++t) {
        if (scl_is_high()) {
            return true;
        }
        I2C_DELAY();
    }

    return false;
}

static bool clock_high(void)
{
    scl_release();
    I2C_DELAY();
    return wait_scl_high();
}

static void i2c_stop(void)
{
    sda_low();
    I2C_DELAY();
    (void) clock_high();
    I2C_DELAY();
    sda_release();
    I2C_DELAY();
}

static bool i2c_start(void)
{
    sda_release();
    scl_release();
    I2C_DELAY();

    if (!wait_scl_high()) {
        i2c_stop();
        return false;
    }

    sda_low();
    I2C_DELAY();
    scl_low();
    I2C_DELAY();
    return true;
}

static bool i2c_wait_ack(void)
{
    sda_release();

    if (!clock_high()) {
        i2c_stop();
        return false;
    }

    for (uint16_t t = 0U; t < SOFT_I2C_ACK_TIMEOUT; ++t) {
        if (!sda_is_high()) {
            scl_low();
            I2C_DELAY();
            return true;
        }
        I2C_DELAY();
    }

    scl_low();
    i2c_stop();
    return false;
}

static bool i2c_write_byte(uint8_t data)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        scl_low();
        if ((data & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }

        I2C_DELAY();
        if (!clock_high()) {
            i2c_stop();
            return false;
        }
        scl_low();
        data <<= 1;
        I2C_DELAY();
    }

    return true;
}

static bool i2c_send_ack(bool ack)
{
    scl_low();
    if (ack) {
        sda_low();
    } else {
        sda_release();
    }

    I2C_DELAY();
    if (!clock_high()) {
        i2c_stop();
        return false;
    }
    scl_low();
    sda_release();
    I2C_DELAY();
    return true;
}

static bool i2c_read_byte(uint8_t *data, bool ack)
{
    uint8_t value = 0U;

    if (data == NULL) {
        return false;
    }

    sda_release();

    for (uint8_t i = 0U; i < 8U; ++i) {
        value <<= 1;
        scl_low();
        I2C_DELAY();

        if (!clock_high()) {
            i2c_stop();
            return false;
        }
        if (sda_is_high()) {
            value |= 1U;
        }
        scl_low();
        I2C_DELAY();
    }

    *data = value;
    return i2c_send_ack(ack);
}

static void i2c_bus_recovery(void)
{
    sda_release();
    scl_release();
    delay_us(50U);

    if (sda_is_high()) {
        return;
    }

    for (uint8_t i = 0U; i < SOFT_I2C_RECOVERY_CLOCKS; ++i) {
        scl_low();
        I2C_DELAY();
        scl_release();
        I2C_DELAY();
        if (sda_is_high()) {
            break;
        }
    }

    i2c_stop();
}

void SoftI2C_Init(void)
{
    /*
     * SysConfig initializes PA0/PA1 as GPIO. Keep them in released/high state
     * before the first transaction, then recover a half-finished transaction if
     * a previous reset happened while the MPU6050 was driving SDA.
     */
    sda_input_pullup();
    scl_input_pullup();
    delay_ms(5U);
    i2c_bus_recovery();
}

void SoftI2C_TestReleaseBoth(void)
{
    sda_release();
    scl_release();
}

void SoftI2C_TestPullSdaLow(void)
{
    sda_low();
}

void SoftI2C_TestPullSclLow(void)
{
    scl_low();
}

bool SoftI2C_TestReadSda(void)
{
    return sda_is_high();
}

bool SoftI2C_TestReadScl(void)
{
    return scl_is_high();
}

bool SoftI2C_ProbeAddress(uint8_t addr)
{
    bool ok = i2c_start() &&
              i2c_write_byte((uint8_t) (addr << 1)) &&
              i2c_wait_ack();

    i2c_stop();
    return ok;
}

bool SoftI2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t data)
{
    bool ok = i2c_start() &&
              i2c_write_byte((uint8_t) (addr << 1)) &&
              i2c_wait_ack() &&
              i2c_write_byte(reg) &&
              i2c_wait_ack() &&
              i2c_write_byte(data) &&
              i2c_wait_ack();

    i2c_stop();
    return ok;
}

bool SoftI2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data)
{
    return SoftI2C_ReadBytes(addr, reg, data, 1U);
}

bool SoftI2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if ((buf == NULL) || (len == 0U)) {
        return false;
    }

    if (!(i2c_start() &&
          i2c_write_byte((uint8_t) (addr << 1)) &&
          i2c_wait_ack() &&
          i2c_write_byte(reg) &&
          i2c_wait_ack() &&
          i2c_start() &&
          i2c_write_byte((uint8_t) ((addr << 1) | 1U)) &&
          i2c_wait_ack())) {
        i2c_stop();
        return false;
    }

    for (uint8_t i = 0U; i < len; ++i) {
        bool ack = (i + 1U) < len;
        if (!i2c_read_byte(&buf[i], ack)) {
            i2c_stop();
            return false;
        }
    }

    i2c_stop();
    return true;
}
