#ifndef HCAR_LINE_SENSOR_H
#define HCAR_LINE_SENSOR_H
#include <stdbool.h>
float LineSensor_GetError(void); bool LineSensor_IsValid(void);
bool LineSensor_IsEndpoint(void); bool LineSensor_IsLost(void);
#endif
