#ifndef HCAR_CONTROL_H
#define HCAR_CONTROL_H
#include <stdbool.h>

typedef enum { MOTION_IDLE, MOTION_DISTANCE, MOTION_HEADING, MOTION_LINE, MOTION_DONE, MOTION_FAULT } MotionState;

/* 运动控制接口 */
void Motion_Init(void);
void Motion_Update10ms(void);
void Motion_Stop(void);
void Motion_DriveDistance(float metres, float speed_mps);
void Motion_DriveHeading(float metres, float speed_mps, float yaw_deg);
void Motion_FollowLine(float speed_mps);
MotionState Motion_GetState(void);
bool Motion_IsComplete(void);
bool Motion_HasFault(void);

/* 参数调节接口 */
void Motion_SetSpeedGains(float kp, float ki);
void Motion_SetHeadingGains(float kp, float kd, float limit);
void Motion_SetLineGains(float kp, float limit);

#endif
