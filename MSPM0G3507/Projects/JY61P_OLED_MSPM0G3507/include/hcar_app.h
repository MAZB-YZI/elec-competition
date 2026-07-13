#ifndef HCAR_APP_H
#define HCAR_APP_H
#include "route_fsm.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HCAR_TEST_NONE,
    HCAR_TEST_MOTOR,
    HCAR_TEST_ENCODER,
    HCAR_TEST_MPU6050,
    HCAR_TEST_BUZZER
} HCarSelfTest;

typedef enum { HCAR_APP_SAFE, HCAR_APP_READY, HCAR_APP_RUNNING, HCAR_APP_FAULT } HCarAppState;

bool HCarApp_Init(void);
void HCarApp_Tick1msISR(void);
void HCarApp_RunPending(void);
bool HCarApp_StartRoute(RouteTest test);
bool HCarApp_StartSelfTest(HCarSelfTest test);
void HCarApp_EmergencyStop(void);
HCarAppState HCarApp_GetState(void);
HCarSelfTest HCarApp_GetSelfTest(void);
uint32_t HCarApp_GetMilliseconds(void);

#endif
