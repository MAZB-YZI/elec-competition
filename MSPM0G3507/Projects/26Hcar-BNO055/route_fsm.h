#ifndef __ROUTE_FSM_H__
#define __ROUTE_FSM_H__

#include <stdbool.h>
#include <stdint.h>

/* ── 运行模式 ── */
typedef enum {
    ROUTE_MODE_STRAIGHT   = 0,  /* 直走校准 */
    ROUTE_MODE_LINE_TEST  = 1,  /* 连续巡线 */
    ROUTE_MODE_H_LAP      = 2,  /* 第二问：一圈停车 */
    ROUTE_MODE_FINISH_TEST = 3, /* A线测试 */
    ROUTE_MODE_Q4_AB      = 4,  /* 第四问：A→B */
    ROUTE_MODE_Q5_LAP     = 5,  /* 第五问：一圈通过A */
    ROUTE_MODE_CALIBRATE  = 6   /* 编码器里程标定 */
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
bool Route_Update5ms(uint32_t now_ms, float yaw_deg, uint8_t gray_raw);

/* ── 模式设置 ── */
void Route_SetMode(RouteMode_t mode);
RouteMode_t Route_GetMode(void);

/* ── 巡线参数 ── */
void  Route_SetKp(float kp);
void  Route_SetKd(float kd);
void  Route_SetBasePwm(int16_t pwm);
void  Route_SetOutputLim(int16_t lim);
void  Route_SetTrim(int16_t trim);
void  Route_SetDeadZone(int16_t dz);
void  Route_SetSlewStep(int16_t step);
float Route_GetKp(void);
float Route_GetKd(void);
int16_t Route_GetBasePwm(void);
int16_t Route_GetOutputLim(void);
int16_t Route_GetTrim(void);
int16_t Route_GetDeadZone(void);
int16_t Route_GetSlewStep(void);

/* ── 停车参数 ── */
void  Route_SetFinishMinDist(float cm);
void  Route_SetFinishMaxDist(float cm);
void  Route_SetBlackMin(uint8_t val);
void  Route_SetFinishConfirmTicks(uint8_t ticks);
void  Route_SetBrakeDurationMs(uint16_t ms);
void  Route_SetLostMs(uint16_t ms);
void  Route_SetStartClearCm(float cm);
float Route_GetFinishMinDist(void);
float Route_GetFinishMaxDist(void);
uint8_t  Route_GetBlackMin(void);
uint8_t  Route_GetFinishConfirmTicks(void);
uint16_t Route_GetBrakeDurationMs(void);
uint16_t Route_GetLostMs(void);
float    Route_GetStartClearCm(void);

/* ── 时间参数 ── */
void     Route_SetLapTargetMs(uint32_t ms);
void     Route_SetTimeWindowMs(uint32_t ms);
void     Route_SetSlowRatio(float ratio);
void     Route_SetSlowAheadMs(uint32_t ms);
void     Route_SetFallbackAheadMs(uint32_t ms);
void     Route_SetTimeoutMs(uint32_t ms);
uint32_t Route_GetLapTargetMs(void);
uint32_t Route_GetTimeWindowMs(void);
float    Route_GetSlowRatio(void);
uint32_t Route_GetSlowAheadMs(void);
uint32_t Route_GetFallbackAheadMs(void);
uint32_t Route_GetTimeoutMs(void);

/* ── 运行状态查询 ── */
bool     Route_IsActive(void);
bool     Route_IsFinished(void);
uint32_t Route_GetElapsedMs(void);
float    Route_GetDistanceCm(void);
RouteFinishReason_t Route_GetFinishReason(void);
const char *Route_GetStateName(void);
const char *Route_GetFinishReasonStr(void);
int16_t  Route_GetLineError(void);
int16_t  Route_GetLineSteer(void);

/* ── PEAK 记录 ── */
uint8_t  Route_GetPeakBc(void);
uint8_t  Route_GetPeakRaw(void);
uint32_t Route_GetPeakMs(void);
float    Route_GetPeakDist(void);

/* ── Q4 参数 ── */
void  Route_SetQ4Pwm(int16_t v);
void  Route_SetQ4RampMs(uint16_t v);
void  Route_SetQ4ArmCm(float v);
void  Route_SetQ4BCm(float v);
void  Route_SetQ4Hkp(float v);
void  Route_SetQ4Hlim(int16_t v);
int16_t  Route_GetQ4Pwm(void);
uint16_t Route_GetQ4RampMs(void);
float    Route_GetQ4ArmCm(void);
float    Route_GetQ4BCm(void);
float    Route_GetQ4Hkp(void);
int16_t  Route_GetQ4Hlim(void);
uint32_t Route_GetQ4AbTimeMs(void);

/* ── Q5 参数 ── */
void  Route_SetQ5Pwm(int16_t v);
void  Route_SetQ5RampMs(uint16_t v);
void  Route_SetQ5OffsetCm(float v);
void  Route_SetQ5PostCm(float v);
void  Route_SetQ5StopMs(uint16_t v);
void  Route_SetQ5TimeoutMs(uint32_t v);
void  Route_SetQ5DetectMinMs(uint32_t v);
void  Route_SetQ5DetectMaxMs(uint32_t v);
int16_t  Route_GetQ5Pwm(void);
uint16_t Route_GetQ5RampMs(void);
float    Route_GetQ5OffsetCm(void);
float    Route_GetQ5PostCm(void);
uint16_t Route_GetQ5StopMs(void);
uint32_t Route_GetQ5TimeoutMs(void);
uint32_t Route_GetQ5DetectMinMs(void);
uint32_t Route_GetQ5DetectMaxMs(void);
uint32_t Route_GetQ5LapTimeMs(void);
bool     Route_GetQ5PassedA(void);

/* ── 蓝牙 START/STOP 请求 ── */
extern volatile bool g_route_start_request;
extern volatile bool g_route_stop_request;

#endif /* __ROUTE_FSM_H__ */
