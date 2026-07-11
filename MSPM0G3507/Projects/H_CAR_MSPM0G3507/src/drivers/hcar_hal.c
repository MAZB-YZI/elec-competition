#include "hcar_hal.h"

#include "encoder.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

bool HCarHal_Init(void)
{
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
        if (in1) DL_GPIO_setPins(MOTOR1_AIN1_PORT, MOTOR1_AIN1_AIN1_PIN);
        else DL_GPIO_clearPins(MOTOR1_AIN1_PORT, MOTOR1_AIN1_AIN1_PIN);

        if (in2) DL_GPIO_setPins(MOTOR1_AIN2_PORT, MOTOR1_AIN2_AIN2_PIN);
        else DL_GPIO_clearPins(MOTOR1_AIN2_PORT, MOTOR1_AIN2_AIN2_PIN);
    } else {
        if (in1) DL_GPIO_setPins(MOTOR2_BIN1_PORT, MOTOR2_BIN1_BIN1_PIN);
        else DL_GPIO_clearPins(MOTOR2_BIN1_PORT, MOTOR2_BIN1_BIN1_PIN);

        if (in2) DL_GPIO_setPins(MOTOR2_BIN2_PORT, MOTOR2_BIN2_BIN2_PIN);
        else DL_GPIO_clearPins(MOTOR2_BIN2_PORT, MOTOR2_BIN2_BIN2_PIN);
    }
}

void HCarHal_SetMotorDuty(bool left, uint16_t duty)
{
    (void) left;
    (void) duty;
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

bool HCarHal_I2CWrite(uint8_t address, const uint8_t *data, size_t length,
    uint32_t timeout_ticks)
{
    if ((data == 0) || (length == 0U)) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    DL_I2C_setTargetAddress(I2C_OLED_INST, address);

    for (size_t i = 0U; i < length; ++i) {
        DL_I2C_transmitControllerData(I2C_OLED_INST, data[i]);
    }

    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout_ticks-- == 0U) {
            return false;
        }
    }

    return true;
}

bool HCarHal_I2CWriteRead(uint8_t address, const uint8_t *tx, size_t tx_length,
    uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    if ((rx == 0) || (rx_length == 0U)) {
        return false;
    }

    if (!HCarHal_I2CWrite(address, tx, tx_length, timeout_ticks)) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_OLED_INST);
    DL_I2C_setTargetAddress(I2C_OLED_INST, address);

    for (size_t i = 0U; i < rx_length; ++i) {
        rx[i] = DL_I2C_receiveControllerData(I2C_OLED_INST);
    }

    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout_ticks-- == 0U) {
            return false;
        }
    }

    return true;
}

void Motor_PlatformSetDirection(MotorId motor, MotorDirection direction)
{
    bool in1 = false;
    bool in2 = false;

    switch (direction) {
    case MOTOR_DIR_FORWARD:
        in1 = true;
        break;
    case MOTOR_DIR_REVERSE:
        in2 = true;
        break;
    case MOTOR_DIR_BRAKE:
        in1 = true;
        in2 = true;
        break;
    case MOTOR_DIR_COAST:
    default:
        break;
    }

    HCarHal_SetMotorDirection(motor == MOTOR_LEFT, in1, in2);
}

void Motor_PlatformSetDuty(MotorId motor, uint16_t duty)
{
    HCarHal_SetMotorDuty(motor == MOTOR_LEFT, duty);
}

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data, size_t length,
    uint32_t timeout_ticks)
{
    return HCarHal_I2CWrite(address, data, length, timeout_ticks);
}

bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    return HCarHal_I2CWriteRead(address, tx, tx_length, rx, rx_length,
        timeout_ticks);
}

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
