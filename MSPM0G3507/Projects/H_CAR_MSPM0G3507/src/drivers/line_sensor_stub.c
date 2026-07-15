#include "line_sensor.h"
#if defined(__GNUC__) || defined(__clang__)
#define WEAK __attribute__((weak))
#else
#define WEAK
#endif
WEAK float LineSensor_GetError(void){return 0;} WEAK bool LineSensor_IsValid(void){return false;}
WEAK bool LineSensor_IsEndpoint(void){return false;} WEAK bool LineSensor_IsLost(void){return true;}
