/**
 * route_fsm.c — H 题专用状态机
 *
 * 整圈灰度巡线 + A 点横线识别 + 超程/超时保护
 * START/STOP 请求由 Route_Update5ms 统一处理
 * 计时由 Route 内部管理
 */

#include "route_fsm.h"
#include "buzzer.h"
#include "motor.h"       /* Motor_* + Encoder_* + PID_* */

/* ── H 题距离/时间常量 ── */
#define H_LAP_THEORY_CM          614.2f
#define H_START_CLEAR_CM          25.0f
#define H_FINISH_WINDOW_MIN_CM   520.0f
#define H_FINISH_WINDOW_MAX_CM   620.0f
#define H_RUN_TIMEOUT_MS       20000U   /* 默认20s 安全停车 */
#define H_FINISH_CONFIRM_TICKS     2U   /* 2×5ms = 10ms 确认 */
#define H_BRAKE_DURATION_TICKS    20U   /* 100ms 刹车 */
#define H_LOST_MAX_TICKS        100U   /* 500ms 丢线保护 */
#define H_LAP_TARGET_MS        17500U   /* 默认标定一圈时间 */
#define H_SLOWDOWN_AHEAD_MS     1500U   /* 提前1.5s开始降速 */
#define H_TIME_WINDOW_MS        1000U   /* 实测停车约18.23s，时间窗口 ±1.0s */
#define H_FALLBACK_AHEAD_MS     1200U   /* 横线失效后备停车提前量 */
#define H_STATIONARY_TICKS        10U   /* 编码器确认静止 50ms */
#define H_SLOWDOWN_RATIO          0.7f  /* 降速比例 */

#define POINT_BEEP_MS             200U

/* ── 状态枚举 ── */
typedef enum {
    ROUTE_STATE_STOPPED = 0,
    ROUTE_STATE_LEAVE_START,
    ROUTE_STATE_FOLLOWING,
    ROUTE_STATE_BRAKING,
    ROUTE_STATE_FINISHED,
    ROUTE_STATE_FAULT
} RouteState_t;

/* ── 蓝牙 START/STOP 请求标志 ── */
volatile bool g_route_start_request = false;
volatile bool g_route_stop_request  = false;

/* ── 启动 pending 标志 ── */
static volatile bool g_start_pending = false;

/* ── 全局状态 ── */
static RouteMode_t  g_mode;
static RouteState_t g_state;

/* ── 计时 (Route 内部管理) ── */
static uint32_t g_run_start_ms;
static uint32_t g_run_elapsed_ms;
static uint32_t g_run_finish_ms;

/* ── 终点检测参数 ── */
static float    g_finish_min_cm;
static float    g_finish_max_cm;
static uint8_t  g_finish_confirm_ticks;
static uint16_t g_brake_duration_ticks;
static uint32_t g_run_timeout_ms;
static uint32_t g_lap_target_ms;         /* 标定一圈时间 */
static uint32_t g_slowdown_ms;           /* 降速时间点 */
static uint32_t g_finish_time_min_ms;    /* 时间窗口下限 */
static uint32_t g_finish_time_max_ms;    /* 时间窗口上限 */
static uint32_t g_fallback_stop_ms;      /* 横线失效后备停车时间 */

/* ── 终点检测状态 ── */
static bool     g_finish_armed;
static uint8_t  g_finish_black_ticks;
static uint16_t g_lost_ticks;
static uint16_t g_brake_ticks;
static RouteFinishReason_t g_finish_reason;

/* ── PEAK 记录（调试用） ── */
static uint8_t  g_peak_bc;
static uint8_t  g_peak_raw;
static uint32_t g_peak_ms;
static float    g_peak_dist;

/* ── 灰度巡线参数 (蓝牙可调) ── */
static volatile float   g_KP         = 1.8f;
static volatile float   g_KD         = 0.0f;
static volatile int16_t g_BASE_PWM   = 1200;
static volatile int16_t g_OUTPUT_LIM = 1500;
static volatile int16_t g_TRIM       = 0;     /* 左右轮速矫正: 正=左快 */

/* ── 灰度巡线内部状态 ── */
#define DEAD_ZONE        3
#define STEER_SLEW_STEP  75
static const int16_t WEIGHT[8] = { 200, 140, 75, 40, -40, -75, -140, -200 };

static int16_t  g_last_steer;
static int16_t  g_last_pos_ctrl;
static uint32_t g_lost_cnt;

/* ── 导出给 OLED 显示 ── */
static volatile int16_t g_line_error;
static volatile int16_t g_line_steer;

/* ── 时间窗口计算 ── */
static void RecalcTimeWindows(void)
{
    g_slowdown_ms        = g_lap_target_ms - H_SLOWDOWN_AHEAD_MS;
    g_finish_time_min_ms = g_lap_target_ms - H_TIME_WINDOW_MS;
    g_finish_time_max_ms = g_lap_target_ms + H_TIME_WINDOW_MS;
    g_fallback_stop_ms   = g_lap_target_ms + H_FALLBACK_AHEAD_MS;
}

/* ── 内部工具 ── */

static int16_t ClampPwm(int32_t v)
{
    if (v > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (v < 0) return 0;
    return (int16_t)v;
}

static uint8_t CountBlack(uint8_t raw)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        count += (raw >> i) & 1U;
    }
    return count;
}

/**
 * 灰度巡线核心 —— 两个模式共用
 *
 * 输入: raw  8 路灰度原始值 (bit=1 表示黑)
 * 输出: 电机已驱动
 */
static void LineFollow_Update(uint8_t raw)
{
    int8_t s[8];
    for (uint8_t i = 0; i < 8; i++) s[i] = (raw >> i) & 1;

    int16_t pos = 0;
    for (uint8_t i = 0; i < 8; i++) pos += WEIGHT[i] * s[i];

    int16_t base = g_BASE_PWM;
    int16_t lim  = g_OUTPUT_LIM;
    int16_t steer = 0;

    bool has_line = (s[0] || s[1] || s[2] || s[3] ||
                     s[4] || s[5] || s[6] || s[7]);
    bool all_black = (raw == 0xFF);

    if (has_line && !all_black) {
        /* 正常巡线 */
        int16_t pos_ctrl = pos;
        if (pos_ctrl > -DEAD_ZONE && pos_ctrl < DEAD_ZONE) {
            pos_ctrl = 0;
        }

        int16_t d_pos = pos_ctrl - g_last_pos_ctrl;
        steer = (int16_t)(-((float)pos_ctrl * g_KP + (float)d_pos * g_KD));
        g_last_pos_ctrl = pos_ctrl;
        g_line_error = pos_ctrl;

        if (steer >  lim) steer =  lim;
        if (steer < -lim) steer = -lim;

        int16_t delta = steer - g_last_steer;
        if (delta >  STEER_SLEW_STEP) steer = g_last_steer + STEER_SLEW_STEP;
        if (delta < -STEER_SLEW_STEP) steer = g_last_steer - STEER_SLEW_STEP;

        g_last_steer = steer;
        g_lost_cnt   = 0;
    } else {
        /* 全白或全黑: 保持上次转向, 超时停车 */
        g_last_pos_ctrl = 0;
        steer = g_last_steer;
        g_line_error = 0;
        if (++g_lost_cnt > H_LOST_MAX_TICKS) {
            Motor_Stop();
            steer = 0;
            g_lost_cnt = 0;
        }
    }

    Motor_SetLeftSpeed(ClampPwm((int32_t)base + steer + g_TRIM));
    Motor_SetRightSpeed(ClampPwm((int32_t)base - steer - g_TRIM));
    g_line_steer = steer;
}

/* ── 重置运行状态 ── */
static void ResetRunState(uint32_t now_ms)
{
    Encoder_ResetDistance();

    g_run_start_ms   = now_ms;
    g_run_elapsed_ms = 0;
    g_run_finish_ms  = 0;

    g_finish_armed       = false;
    g_finish_black_ticks = 0;
    g_lost_ticks         = 0;
    g_brake_ticks        = 0;
    g_finish_reason      = FINISH_REASON_NONE;
    g_peak_bc   = 0;
    g_peak_raw  = 0;
    g_peak_ms   = 0;
    g_peak_dist = 0.0f;

    g_last_steer    = 0;
    g_last_pos_ctrl = 0;
    g_lost_cnt      = 0;
    g_line_error    = 0;
    g_line_steer    = 0;
}

/* ── 5ms 主循环 ── */

bool Route_Update5ms(uint32_t now_ms, float yaw_deg, uint8_t gray_raw)
{
    (void)yaw_deg;

    Buzzer_Update5ms();

    /* ── 处理 START/STOP 请求 (STOP 优先，Route 是唯一处理者) ── */
    if (g_route_stop_request) {
        g_route_stop_request = false;
        g_start_pending = false;
        Route_Stop();
    } else if (g_route_start_request || g_start_pending) {
        g_route_start_request = false;
        g_start_pending = false;
        /* 执行启动 */
        Motor_Stop();
        ResetRunState(now_ms);
        if (g_mode == ROUTE_MODE_LINE_TEST) {
            g_state = ROUTE_STATE_FOLLOWING;
            Buzzer_Beep(POINT_BEEP_MS);
        } else if (g_mode == ROUTE_MODE_H_LAP) {
            g_state = ROUTE_STATE_LEAVE_START;
            Buzzer_Beep(POINT_BEEP_MS);
        } else if (g_mode == ROUTE_MODE_FINISH_TEST) {
            g_state = ROUTE_STATE_FOLLOWING;
            Buzzer_Beep(POINT_BEEP_MS);
        } else if (g_mode == ROUTE_MODE_STRAIGHT) {
            g_state = ROUTE_STATE_FOLLOWING;
            Buzzer_Beep(POINT_BEEP_MS);
        } else {
            g_state = ROUTE_STATE_STOPPED;
        }
    }

    switch (g_state) {

    /* ── 停车 ── */
    case ROUTE_STATE_STOPPED:
        Motor_Stop();
        return false;

    /* ── 离开起点: 巡线但禁止终点检测 ── */
    case ROUTE_STATE_LEAVE_START:
    {
        LineFollow_Update(gray_raw);

        float dist = Encoder_GetAverageDistanceCm();
        if (dist >= H_START_CLEAR_CM) {
            g_finish_armed = true;
            g_state = ROUTE_STATE_FOLLOWING;
        }
        /* 更新运行时间 */
        g_run_elapsed_ms = now_ms - g_run_start_ms;
        return true;
    }

    /* ── 正常巡线 + 终点检测 ── */
    case ROUTE_STATE_FOLLOWING:
    {
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_run_start_ms;
        g_run_elapsed_ms = elapsed;

        /* 直走模式: 只用 BASE + TRIM，不巡线 */
        if (g_mode == ROUTE_MODE_STRAIGHT) {
            Motor_SetLeftSpeed(ClampPwm((int32_t)g_BASE_PWM + g_TRIM));
            Motor_SetRightSpeed(ClampPwm((int32_t)g_BASE_PWM - g_TRIM));
            g_line_error = 0;
            g_line_steer = 0;
            return true;
        }

        /* ── 终点前降速 ── */
        if (elapsed >= g_slowdown_ms && g_finish_armed) {
            int16_t slow_base = (int16_t)((float)g_BASE_PWM * H_SLOWDOWN_RATIO);
            /* 用降速后的 base 巡线 */
            int8_t s[8];
            for (uint8_t i = 0; i < 8; i++) s[i] = (gray_raw >> i) & 1;
            int16_t pos = 0;
            for (uint8_t i = 0; i < 8; i++) pos += WEIGHT[i] * s[i];
            bool has_line = (s[0] || s[1] || s[2] || s[3] ||
                             s[4] || s[5] || s[6] || s[7]);
            bool all_black = (gray_raw == 0xFF);
            int16_t steer = 0;
            if (has_line && !all_black) {
                int16_t pos_ctrl = pos;
                if (pos_ctrl > -DEAD_ZONE && pos_ctrl < DEAD_ZONE) pos_ctrl = 0;
                int16_t d_pos = pos_ctrl - g_last_pos_ctrl;
                steer = (int16_t)(-((float)pos_ctrl * g_KP + (float)d_pos * g_KD));
                g_last_pos_ctrl = pos_ctrl;
                g_line_error = pos_ctrl;
                if (steer >  g_OUTPUT_LIM) steer =  g_OUTPUT_LIM;
                if (steer < -g_OUTPUT_LIM) steer = -g_OUTPUT_LIM;
                int16_t delta = steer - g_last_steer;
                if (delta >  STEER_SLEW_STEP) steer = g_last_steer + STEER_SLEW_STEP;
                if (delta < -STEER_SLEW_STEP) steer = g_last_steer - STEER_SLEW_STEP;
                g_last_steer = steer;
                g_lost_cnt = 0;
            } else {
                g_last_pos_ctrl = 0;
                steer = g_last_steer;
                g_line_error = 0;
            }
            g_line_steer = steer;
            Motor_SetLeftSpeed(ClampPwm((int32_t)slow_base + steer + g_TRIM));
            Motor_SetRightSpeed(ClampPwm((int32_t)slow_base - steer - g_TRIM));
        } else {
            /* 正常速度巡线 */
            LineFollow_Update(gray_raw);
        }

        /* ── A 横线检测（时间窗口 + 距离窗口 + 横线确认） ── */
        bool in_time_window = (elapsed >= g_finish_time_min_ms &&
                               elapsed <= g_finish_time_max_ms);
        bool in_dist_window = (dist >= g_finish_min_cm &&
                               dist <= g_finish_max_cm);

        /* 记录峰值黑色通道数（调试用） */
        if (g_finish_armed && in_time_window) {
            uint8_t bc = CountBlack(gray_raw);
            if (bc > g_peak_bc) {
                g_peak_bc   = bc;
                g_peak_raw  = gray_raw;
                g_peak_ms   = elapsed;
                g_peak_dist = dist;
            }
        }

        if (g_finish_armed && in_time_window && in_dist_window) {
            bool candidate = (CountBlack(gray_raw) >= 4U);
            if (candidate) {
                if (g_finish_black_ticks < g_finish_confirm_ticks) {
                    g_finish_black_ticks++;
                }
            } else {
                g_finish_black_ticks = 0;
            }

            if (g_finish_black_ticks >= g_finish_confirm_ticks) {
                g_finish_reason = FINISH_REASON_LINE;
                g_brake_ticks = 0;
                g_state = ROUTE_STATE_BRAKING;
                Motor_Brake();
                return true;
            }
        }

        /* ── 横线失效后备停车（时间到达标定值） ── */
        if (g_finish_armed && elapsed >= g_fallback_stop_ms) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_brake_ticks = 0;
            g_state = ROUTE_STATE_BRAKING;
            Motor_Brake();
            return true;
        }

        /* ── 丢线保护 (只算全白，全黑不算丢线) ── */
        if (gray_raw == 0x00) {
            g_lost_ticks++;
        } else {
            g_lost_ticks = 0;
        }
        if (g_lost_ticks >= H_LOST_MAX_TICKS) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed;
            g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT;
            Motor_Stop();
            return true;
        }

        /* ── 超程保护 ── */
        if (dist >= g_finish_max_cm + 100.0f) {
            g_finish_reason = FINISH_REASON_OVERRUN;
            g_brake_ticks = 0;
            g_state = ROUTE_STATE_FAULT;
            Motor_Brake();
            return true;
        }

        /* ── 安全超时保护 ── */
        if (elapsed >= g_run_timeout_ms) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_run_finish_ms = elapsed;
            g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT;
            Motor_Stop();
            return true;
        }

        return true;
    }

    /* ── 刹车制动 ── */
    case ROUTE_STATE_BRAKING:
        Motor_Brake();
        g_brake_ticks++;
        if (g_brake_ticks >= g_brake_duration_ticks) {
            /* 停车确认后冻结时间 */
            g_run_finish_ms = now_ms - g_run_start_ms;
            g_run_elapsed_ms = g_run_finish_ms;
            Motor_Stop();
            g_state = ROUTE_STATE_FINISHED;
        }
        /* 刹车期间继续更新时间显示 */
        g_run_elapsed_ms = now_ms - g_run_start_ms;
        return true;

    /* ── 完成: 冻结时间 ── */
    case ROUTE_STATE_FINISHED:
        Motor_Stop();
        /* g_run_elapsed_ms 已在 BRAKING 结束时冻结 */
        return true;

    /* ── 故障停车 ── */
    case ROUTE_STATE_FAULT:
        if (g_finish_reason == FINISH_REASON_OVERRUN && g_brake_ticks < g_brake_duration_ticks) {
            Motor_Brake();
            g_brake_ticks++;
            g_run_elapsed_ms = now_ms - g_run_start_ms;
        } else {
            /* 制动完成或非超程故障：冻结时间，停车 */
            if (g_run_finish_ms == 0) {
                g_run_finish_ms = now_ms - g_run_start_ms;
                g_run_elapsed_ms = g_run_finish_ms;
            }
            Motor_Stop();
        }
        return true;

    default:
        Motor_Stop();
        g_state = ROUTE_STATE_STOPPED;
        return false;
    }
}

/* ── 生命周期 ── */

void Route_Init(void)
{
    g_mode  = ROUTE_MODE_H_LAP;   /* 第二问默认模式，仍需 START 才启动 */
    g_state = ROUTE_STATE_STOPPED;

    g_run_start_ms   = 0;
    g_run_elapsed_ms = 0;
    g_run_finish_ms  = 0;

    g_finish_min_cm       = H_FINISH_WINDOW_MIN_CM;
    g_finish_max_cm       = H_FINISH_WINDOW_MAX_CM;
    g_finish_confirm_ticks = H_FINISH_CONFIRM_TICKS;
    g_brake_duration_ticks = H_BRAKE_DURATION_TICKS;
    g_run_timeout_ms       = H_RUN_TIMEOUT_MS;
    g_lap_target_ms        = H_LAP_TARGET_MS;
    RecalcTimeWindows();

    g_finish_armed       = false;
    g_finish_black_ticks = 0;
    g_lost_ticks         = 0;
    g_brake_ticks        = 0;
    g_finish_reason      = FINISH_REASON_NONE;
    g_start_pending      = false;

    g_KP         = 1.8f;
    g_KD         = 0.0f;
    g_BASE_PWM   = 1200;
    g_OUTPUT_LIM = 1500;

    g_last_steer    = 0;
    g_last_pos_ctrl = 0;
    g_lost_cnt      = 0;
    g_line_error    = 0;
    g_line_steer    = 0;
}

void Route_Start(void)
{
    /* 只设 pending，实际启动在 Route_Update5ms 中执行 */
    g_start_pending = true;
}

void Route_Stop(void)
{
    Motor_Stop();
    g_state = ROUTE_STATE_STOPPED;
    g_start_pending = false;
    g_brake_ticks = 0;
    Buzzer_Beep(POINT_BEEP_MS);
}

/* ── 模式设置 ── */

void Route_SetMode(RouteMode_t mode)
{
    if (mode == ROUTE_MODE_STOP || mode == ROUTE_MODE_LINE_TEST ||
        mode == ROUTE_MODE_H_LAP || mode == ROUTE_MODE_FINISH_TEST ||
        mode == ROUTE_MODE_STRAIGHT) {
        g_mode = mode;
    }
}

RouteMode_t Route_GetMode(void) { return g_mode; }

/* ── 灰度巡线参数 ── */

void Route_SetBasePwm(int16_t pwm)
{
    if (pwm < 0) pwm = 0;
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    g_BASE_PWM = pwm;
}

void Route_SetKp(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    g_KP = kp;
}

void Route_SetKd(float kd)
{
    g_KD = kd;
}

void Route_SetOutputLim(int16_t lim)
{
    if (lim < 0) lim = 0;
    if (lim > MOTOR_PWM_MAX) lim = MOTOR_PWM_MAX;
    g_OUTPUT_LIM = lim;
}

int16_t Route_GetBasePwm(void)   { return g_BASE_PWM; }
float   Route_GetKp(void)        { return g_KP; }
float   Route_GetKd(void)        { return g_KD; }
int16_t Route_GetOutputLim(void) { return g_OUTPUT_LIM; }

void Route_SetTrim(int16_t trim)
{
    if (trim > 500) trim = 500;
    if (trim < -500) trim = -500;
    g_TRIM = trim;
}
int16_t Route_GetTrim(void) { return g_TRIM; }

/* ── 终点检测参数 ── */

void Route_SetFinishMinDist(float cm)
{
    if (cm < 100.0f) cm = 100.0f;
    if (cm > g_finish_max_cm - 50.0f) cm = g_finish_max_cm - 50.0f;
    g_finish_min_cm = cm;
}

void Route_SetFinishMaxDist(float cm)
{
    if (cm < g_finish_min_cm + 50.0f) cm = g_finish_min_cm + 50.0f;
    if (cm > 1000.0f) cm = 1000.0f;
    g_finish_max_cm = cm;
}

void Route_SetFinishConfirmTicks(uint8_t ticks)
{
    if (ticks < 1U) ticks = 1U;
    if (ticks > 20U) ticks = 20U;
    g_finish_confirm_ticks = ticks;
}

void Route_SetBrakeDurationMs(uint16_t ms)
{
    g_brake_duration_ticks = ms / 5U;
    if (g_brake_duration_ticks < 1U) g_brake_duration_ticks = 1U;
}

uint16_t Route_GetBrakeDurationMs(void) { return g_brake_duration_ticks * 5U; }
uint8_t  Route_GetFinishConfirmTicks(void) { return g_finish_confirm_ticks; }

float Route_GetFinishMinDist(void) { return g_finish_min_cm; }
float Route_GetFinishMaxDist(void) { return g_finish_max_cm; }

/* ── 运行状态查询 ── */

bool Route_IsActive(void)
{
    switch (g_state) {
    case ROUTE_STATE_LEAVE_START:
    case ROUTE_STATE_FOLLOWING:
    case ROUTE_STATE_BRAKING:
        return true;
    default:
        return false;
    }
}

bool Route_IsFinished(void)
{
    return (g_state == ROUTE_STATE_FINISHED || g_state == ROUTE_STATE_FAULT);
}

uint32_t Route_GetElapsedMs(void)
{
    return g_run_elapsed_ms;
}

float Route_GetDistanceCm(void)
{
    return Encoder_GetAverageDistanceCm();
}

RouteFinishReason_t Route_GetFinishReason(void)
{
    return g_finish_reason;
}

const char *Route_GetStateName(void)
{
    switch (g_state) {
    case ROUTE_STATE_STOPPED:      return "STOP";
    case ROUTE_STATE_LEAVE_START:  return "LVST";
    case ROUTE_STATE_FOLLOWING:    return "FOLL";
    case ROUTE_STATE_BRAKING:      return "BRK ";
    case ROUTE_STATE_FINISHED:     return "DONE";
    case ROUTE_STATE_FAULT:        return "ERR ";
    default:                       return "????";
    }
}

const char *Route_GetFinishReasonStr(void)
{
    switch (g_finish_reason) {
    case FINISH_REASON_LINE:    return "LINE";
    case FINISH_REASON_OVERRUN: return "OVERRUN";
    case FINISH_REASON_TIMEOUT: return "TIMEOUT";
    case FINISH_REASON_LOST:    return "LOST";
    default:                    return "NONE";
    }
}

int16_t Route_GetLineError(void) { return g_line_error; }
int16_t Route_GetLineSteer(void) { return g_line_steer; }

uint8_t  Route_GetPeakBc(void)   { return g_peak_bc; }
uint8_t  Route_GetPeakRaw(void)  { return g_peak_raw; }
uint32_t Route_GetPeakMs(void)   { return g_peak_ms; }
float    Route_GetPeakDist(void)  { return g_peak_dist; }

void Route_SetTimeoutMs(uint32_t ms)
{
    if (ms < 5000U) ms = 5000U;
    if (ms > 60000U) ms = 60000U;
    g_run_timeout_ms = ms;
}
uint32_t Route_GetTimeoutMs(void) { return g_run_timeout_ms; }

void Route_SetLapTargetMs(uint32_t ms)
{
    if (ms < 5000U) ms = 5000U;
    if (ms > 30000U) ms = 30000U;
    g_lap_target_ms = ms;
    RecalcTimeWindows();
    /* 自动同步安全超时 = 后备停车 + 1.3s 余量 */
    g_run_timeout_ms = g_fallback_stop_ms + 1300U;
}
uint32_t Route_GetLapTargetMs(void) { return g_lap_target_ms; }
