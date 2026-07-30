/**
 * @file linkage_calibration.h
 * @brief Linkage angle-to-motor-position calibration
 *
 * Returns error on out-of-range instead of silent clamping.
 * Points must remain sorted by angle_mdeg after SetPoint.
 */
#ifndef LINKAGE_CALIBRATION_H
#define LINKAGE_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t angle_mdeg;
    int32_t motor_position;
} LinkageCalPoint;

/** @brief Initialize calibration table */
void Linkage_Init(void);

/**
 * @brief Convert angle to motor position
 * @return true on success, false if angle outside table range
 */
bool Linkage_AngleToPosition(int16_t angle_mdeg, int32_t *position);

/**
 * @brief Convert motor position to angle
 * @return true on success, false if position outside table range
 */
bool Linkage_PositionToAngle(int32_t position, int16_t *angle_mdeg);

/** @brief Get calibration table pointer */
const LinkageCalPoint *Linkage_GetTable(uint8_t *count);

/**
 * @brief Update a single calibration point
 * @return 0 on success, -1 if index out of range or would break sort order
 */
int Linkage_SetPoint(uint8_t index, int16_t angle, int32_t position);

#endif /* LINKAGE_CALIBRATION_H */
