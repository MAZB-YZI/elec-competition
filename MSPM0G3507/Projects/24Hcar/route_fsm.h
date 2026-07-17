#ifndef __ROUTE_FSM_H__
#define __ROUTE_FSM_H__

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROUTE_MODE_REQ1 = 1,
    ROUTE_MODE_REQ2 = 2,
    ROUTE_MODE_REQ3 = 3,
    ROUTE_MODE_REQ4 = 4,
    ROUTE_MODE_ARC_TEST = 5    /* 单独测圆弧循迹 */
} RouteMode_t;

void Route_Init(void);
void Route_SetMode(RouteMode_t mode);

/* 直线段调参 */
void Route_SetStraightBase(int16_t pwm);
void Route_SetHeadingKp(float kp);
void Route_SetStraightTrim(int16_t trim);
void Route_SetHeadingSign(int8_t sign);

/* 圆弧段调参 */
void Route_SetArcBase(int16_t pwm);
void Route_SetLineKp(float kp);
void Route_SetLineKd(float kd);
void Route_SetArcDistCm(float cm);
void Route_SetArcSearchSpd(float spd);
void Route_SetAcAngle(float deg);
void Route_SetBdAngle(float deg);
void Route_SetTurnKp(float kp);

float Route_GetAcAngle(void);
float Route_GetBdAngle(void);
float Route_GetTurnKp(void);
float Route_GetArcDistCm(void);
float Route_GetLineKp(void);
float Route_GetLineKd(void);
float Route_GetArcSearchSpd(void);

void Route_Start(void);
void Route_Stop(void);
bool Route_Update5ms(float yaw_deg);
bool Route_IsActive(void);
const char *Route_GetStateName(void);

#endif /* __ROUTE_FSM_H__ */
