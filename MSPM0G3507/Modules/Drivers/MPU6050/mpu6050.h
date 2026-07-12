#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MPU6050_I2C_ADDRESS 0x68U
#define MPU6050_I2C_TIMEOUT_TICKS 1000U
#define MPU6050_WHO_AM_I_EXPECTED 0x68U

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050Raw;

bool MPU6050_Init(void);
bool MPU6050_ReadRaw(MPU6050Raw *raw);
bool MPU6050_CalibrateGyro(uint16_t samples);
bool MPU6050_Update(float dt_s);
float MPU6050_GetYaw(void);
float MPU6050_GetGyroZ(void);
float MPU6050_GetGyroZBias(void);
uint8_t MPU6050_GetWhoAmI(void);
uint32_t MPU6050_GetReadErrorCount(void);
void MPU6050_ResetYaw(void);
void MPU6050_ResetYawTo(float yaw_deg);

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data, size_t length,
    uint32_t timeout_ticks);
bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks);

#endif
