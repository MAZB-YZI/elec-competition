#ifndef JY61P_H
#define JY61P_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint16_t yaw_raw;
    float gyro_z_dps;
    uint32_t rx_byte_count;
    uint32_t frame_count;
    uint32_t checksum_error_count;
    uint32_t rx_error_count;
} JY61P_Status;

void JY61P_Init(void);
void JY61P_ResetYaw(void);
void JY61P_ResetYawTo(float yaw_deg);
void JY61P_RequestZeroYaw(void);

bool JY61P_UpdateStatus(JY61P_Status *status);

float JY61P_GetRoll(void);
float JY61P_GetPitch(void);
float JY61P_GetYaw(void);
uint16_t JY61P_GetYawRaw(void);
float JY61P_GetGyroZ(void);
uint32_t JY61P_GetRxByteCount(void);
uint32_t JY61P_GetFrameCount(void);
uint32_t JY61P_GetChecksumErrorCount(void);
uint32_t JY61P_GetRxErrorCount(void);

#endif /* JY61P_H */
