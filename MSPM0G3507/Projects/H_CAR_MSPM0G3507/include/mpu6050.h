#ifndef HCAR_MPU6050_H
#define HCAR_MPU6050_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { int16_t accel_x,accel_y,accel_z,temp,gyro_x,gyro_y,gyro_z; } MPU6050Raw;
bool MPU6050_Init(void); bool MPU6050_ReadRaw(MPU6050Raw *raw);
bool MPU6050_CalibrateGyro(uint16_t samples); bool MPU6050_Update(float dt_s);
float MPU6050_GetYaw(void); void MPU6050_ResetYaw(void);
#endif
