#ifndef __ROUTE_FSM_H__
#define __ROUTE_FSM_H__

#include <stdbool.h>
#include <stdint.h>

/* ── 运行模式 ── */
typedef enum {
    ROUTE_MODE_STOP       = 0,  /* 停车 */
    ROUTE_MODE_LINE_TEST  = 1,  /* 连续巡线，不自动停车 */
    ROUTE_MODE_H_LAP      = 2,  /* H题：绕一圈并在A点停车 */
    ROUTE_MODE_FINISH_TEST = 3  /* 只测试A点横线识别 */
} RouteMode_t;

/* ── 结束原因 ── */
typedef enum {
    FINISH_REASON_NONE    = 0,
    FINISH_REASON_LINE,        /* 正常检测到 A 点横线 */
    FINISH_REASON_OVERRUN,     /* 超程保护 */
    FINISH_REASON_TIMEOUT,     /* 超时保护 */
    FINISH_REASON_LOST         /* 长时间丢线 */
} RouteFinishReason_t;

/* ── 生命周期 ── */
void Route_Init(void);
void Route_Start(void);
void Route_Stop(void);
bool Route_Update5ms(float yaw_deg, uint8_t gray_raw);

/* ── 模式设置 ── */
void Route_SetMode(RouteMode_t mode);
RouteMode_t Route_GetMode(void);

/* ── 灰度巡线参数 ── */
void  Route_SetBasePwm(int16_t pwm);
void  Route_SetKp(float kp);
void  Route_SetKd(float kd);
void  Route_SetOutputLim(int16_t lim);
int16_t Route_GetBasePwm(void);
float Route_GetKp(void);
float Route_GetKd(void);
int16_t Route_GetOutputLim(void);

/* ── 终点检测参数 ── */
void Route_SetFinishMinDist(float cm);
void Route_SetFinishMaxDist(float cm);
void Route_SetFinishConfirmTicks(uint8_t ticks);
void Route_SetBrakeDurationMs(uint16_t ms);
float Route_GetFinishMinDist(void);
float Route_GetFinishMaxDist(void);

/* ── 运行状态查询 ── */
bool     Route_IsActive(void);
bool     Route_IsFinished(void);
uint32_t Route_GetElapsedMs(void);
float    Route_GetDistanceCm(void);
RouteFinishReason_t Route_GetFinishReason(void);
const char *Route_GetStateName(void);
const char *Route_GetFinishReasonStr(void);
int16_t  Route_GetLineError(void);   /* 实际巡线位置误差，中心=0 */
int16_t  Route_GetLineSteer(void);   /* 实际巡线转向量 */

/* ── 蓝牙 START/STOP 请求 (主循环写, ISR 读) ── */
extern volatile bool g_route_start_request;
extern volatile bool g_route_stop_request;

#endif /* __ROUTE_FSM_H__ */
