/*
 * hcar_hal.c - H_CAR 板级适配文件
 * 使用软件 I2C 驱动 MPU6050 (PA0/PA1)
 */

#include "hcar_hal.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "../../../../Modules/Drivers/MPU6050/soft_i2c.h"
#include "ti_msp_dl_config.h"

bool HCarHal_Init(void)
{
    /* 初始化软件 I2C */
    SoftI2C_Init();

    /* 初始化编码器中断 */
    DL_GPIO_clearInterruptStatus(GPIOA,
        ENCODER1_A_ENC1_A_PIN | ENCODER2_A_ENC2_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER1_A_ENC1_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER2_A_ENC2_A_PIN);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    return true;
}

bool HCarHal_Start1msTick(void)
{
    NVIC_ClearPendingIRQ(SYS_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(SYS_TICK_INST_INT_IRQN);
    return true;
}

void HCarHal_Idle(void)
{
    __WFI();
}

void HCarHal_SetMotorDirection(bool left, bool in1, bool in2)
{
    if (left) {
        if (in1) DL_GPIO_setPins(MOTOR_DIR_L_PORT, MOTOR_DIR_L_L_DIR1_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_L_PORT, MOTOR_DIR_L_L_DIR1_PIN);

        if (in2) DL_GPIO_setPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_L_DIR2_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_L_DIR2_PIN);
    } else {
        if (in1) DL_GPIO_setPins(MOTOR_DIR_R_PORT, MOTOR_DIR_R_R_DIR1_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_R_PORT, MOTOR_DIR_R_R_DIR1_PIN);

        if (in2) DL_GPIO_setPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_R_DIR2_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_R_DIR2_PIN);
    }
}

void HCarHal_SetMotorDuty(bool left, uint16_t duty)
{
    (void) left;
    (void) duty;
    /* TODO: 需要配置 PWM */
}

uint16_t HCarHal_GetMotorPeriod(void)
{
    return 4000U;
}

bool HCarHal_ReadEncoderB(bool left)
{
    if (left) {
        return (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_14) != 0U);
    }
    return (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_27) != 0U);
}

void HCarHal_SetBuzzer(bool on)
{
    if (on) DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_CTRL_PIN);
    else DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_CTRL_PIN);
}

void HCarHal_SetStatusLed(bool on)
{
    if (on) DL_GPIO_setPins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
    else DL_GPIO_clearPins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
}

/* ========== MPU6050 软件 I2C 平台函数 ========== */

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data, size_t length,
    uint32_t timeout_ticks)
{
    (void) timeout_ticks;

    if ((data == NULL) || (length == 0U)) {
        return false;
    }

    /* 对于单字节写入，使用 SoftI2C_WriteReg */
    if (length == 2U) {
        return SoftI2C_WriteReg(address, data[0], data[1]);
    }

    /* 多字节写入：逐字节写入 */
    for (size_t i = 1; i < length; i++) {
        if (!SoftI2C_WriteReg(address, data[0] + (uint8_t)(i - 1), data[i])) {
            return false;
        }
    }

    return true;
}

bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    (void) timeout_ticks;

    if ((tx == NULL) || (tx_length == 0U) || (rx == NULL) || (rx_length == 0U)) {
        return false;
    }

    /* 对于单字节读取，使用 SoftI2C_ReadReg */
    if (rx_length == 1U) {
        return SoftI2C_ReadReg(address, tx[0], rx);
    }

    /* 多字节读取，使用 SoftI2C_ReadBytes */
    return SoftI2C_ReadBytes(address, tx[0], rx, (uint8_t) rx_length);
}

/* ========== 编码器中断 ========== */

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        ENCODER1_A_ENC1_A_PIN | ENCODER2_A_ENC2_A_PIN);

    if ((pending & ENCODER1_A_ENC1_A_PIN) != 0U) {
        Encoder_OnLeftAEdge();
    }
    if ((pending & ENCODER2_A_ENC2_A_PIN) != 0U) {
        Encoder_OnRightAEdge();
    }

    DL_GPIO_clearInterruptStatus(GPIOA, pending);
}
