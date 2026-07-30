/**
 * route_fsm.c — H 题专用状态机
 *
 * 整圈灰度巡线 + A 点横线识别 + 超程/超时保护
 */

#include "route_fsm.h"
#include "buzzer.h"
#include "motor.h"       /* Motor_* + Encoder_* + PID_* */

/* ── H 题距离/时间常量 ── */
#define H_LAP_THEORY_CM          614.2f
#define H_START_CLEAR_CM          25.0f
#define H_FINISH_WINDOW_MIN_CM   520.0f
#define H_FINISH_WINDOW_MAX_CM   700.0f
#define H_RUN_TIMEOUT_MS       25000U
#define H_FINISH_CONFIRM_TICKS     4U
#define H_BRAKE_DURATION_TICKS    20U   /* 100 ms */
#define H_LOST_MAX_TICKS        100U   /* 500 ms 丢线保护 */

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

/* ── 全局状态 ── */
static RouteMode_t  g_mode;
static RouteState_t g_state;

/* ── 运行计时/距离 ── */
static uint32_t g_run_start_ms;
static uint32_t g_run_finish_ms;
static float    g_finish_min_cm;
static float    g_finish_max_cm;
static uint8_t  g_finish_confirm_ticks;
static uint16_t g_brake_duration_ticks;

/* ── 终点检测 ── */
static bool     g_finish_armed;
static uint8_t  g_finish_black_ticks;
static uint16_t g_lost_ticks;
static RouteFinishReason_t g_finish_reason;

/* ── 灰度巡线参数 (蓝牙可调) ── */
static volatile float   g_KP         = 1.8f;
static volatile float   g_KD         = 0.0f;
static volatile int16_t g_BASE_PWM   = 700;
static volatile int16_t g_OUTPUT_LIM = 900;

/* ── 灰度巡线内部状态 ── */
#define DEAD_ZONE        3
#define STEER_SLEW_STEP  75
static const int16_t WEIGHT[8] = { 200, 140, 75, 40, -40, -75, -140, -200 };

static int16_t  g_last_steer;
static int16_t  g_last_pos_ctrl;
static uint32_t g_lost_cnt;

/* 导出给 OLED 显示 */
static volatile int16_t g_line_error;
static volatile int16_t g_line_steer;

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
 *
 * 内部使用 g_KP / g_KD / g_BASE_PWM / g_OUTPUT_LIM
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

        /* 转向变化率限制 */
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

    Motor_SetLeftSpeed(ClampPwm((int32_t)base + steer));
    Motor_SetRightSpeed(ClampPwm((int32_t)base - steer));
    g_line_steer = steer;
}

/* ── 5ms 主循环 ── */

bool Route_Update5ms(float yaw_deg, uint8_t gray_raw)
{
    (void)yaw_deg;  /* H 题模式暂不使用航向角控制电机 */

    Buzzer_Update5ms();

    /* ── 处理蓝牙 START/STOP 请求 (STOP 优先) ── */
    if (g_route_stop_request) {
        g_route_stop_request = false;
        Route_Stop();
    } else if (g_route_start_request) {
        g_route_start_request = false;
        Route_Start();
    }

    switch (g_state) {

    /* ── 停车: 不控电机, 返回 false 让 main ISR 可跑普通巡线 ── */
    case ROUTE_STATE_STOPPED:
        Motor_Stop();
        return false;

    /* ── 离开起点: 灰度巡线但禁止终点检测 ── */
    case ROUTE_STATE_LEAVE_START:
    {
        LineFollow_Update(gray_raw);

        float dist = Encoder_GetAverageDistanceCm();
        if (dist >= H_START_CLEAR_CM) {
            g_finish_armed = true;
            g_state = ROUTE_STATE_FOLLOWING;
        }
        return true;
    }

    /* ── 正常巡线 + 终点检测 ── */
    case ROUTE_STATE_FOLLOWING:
    {
        LineFollow_Update(gray_raw);

        float dist = Encoder_GetAverageDistanceCm();

        /* ── 终点窗口内: 检测 A 点横线 ── */
        if (g_finish_armed && dist >= g_finish_min_cm) {
            bool candidate = (CountBlack(gray_raw) >= 6U);
            if (candidate) {
                if (g_finish_black_ticks < g_finish_confirm_ticks) {
                    g_finish_black_ticks++;
                }
            } else {
                g_finish_black_ticks = 0;
            }

            if (g_finish_black_ticks >= g_finish_confirm_ticks) {
                g_finish_reason = FINISH_REASON_LINE;
                g_state = ROUTE_STATE_BRAKING;
                Motor_Brake();
                g_brake_duration_ticks = 0;
                return true;
            }
        }

        /* ── 超程保护 ── */
        if (dist >= g_finish_max_cm) {
            g_finish_reason = FINISH_REASON_OVERRUN;
            g_state = ROUTE_STATE_BRAKING;
            Motor_Brake();
            g_brake_duration_ticks = 0;
            return true;
        }

        /* ── 丢线保护 ── */
        bool has_line = (gray_raw != 0x00) && (gray_raw != 0xFF);
        if (!has_line) {
            g_lost_ticks++;
        } else {
            g_lost_ticks = 0;
        }
        if (g_lost_ticks >= H_LOST_MAX_TICKS) {
            g_finish_reason = FINISH_REASON_LOST;
            g_state = ROUTE_STATE_FAULT;
            Motor_Stop();
            return true;
        }

        return true;
    }

    /* ── 刹车制动 ── */
    case ROUTE_STATE_BRAKING:
        g_brake_duration_ticks++;
        if (g_brake_duration_ticks >= H_BRAKE_DURATION_TICKS) {
            Motor_Stop();
            g_state = ROUTE_STATE_FINISHED;
        }
        return true;

    /* ── 完成: 冻结时间/距离 ── */
    case ROUTE_STATE_FINISHED:
        Motor_Stop();
        return true;

    /* ── 故障停车 ── */
    case ROUTE_STATE_FAULT:
        Motor_Stop();
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
    g_mode  = ROUTE_MODE_STOP;
    g_state = ROUTE_STATE_STOPPED;

    g_run_start_ms  = 0;
    g_run_finish_ms = 0;

    g_finish_min_cm       = H_FINISH_WINDOW_MIN_CM;
    g_finish_max_cm       = H_FINISH_WINDOW_MAX_CM;
    g_finish_confirm_ticks = H_FINISH_CONFIRM_TICKS;
    g_brake_duration_ticks = H_BRAKE_DURATION_TICKS;

    g_finish_armed       = false;
    g_finish_black_ticks = 0;
    g_lost_ticks         = 0;
    g_finish_reason      = FINISH_REASON_NONE;

    g_KP         = 1.8f;
    g_KD         = 0.0f;
    g_BASE_PWM   = 700;
    g_OUTPUT_LIM = 900;

    g_last_steer    = 0;
    g_last_pos_ctrl = 0;
    g_lost_cnt      = 0;
    g_line_error    = 0;
    g_line_steer    = 0;
}

void Route_Start(void)
{
    Motor_Stop();
    Encoder_ResetDistance();

    g_run_start_ms       = 0;   /* 由 ISR 设置实际时间 */
    g_run_finish_ms      = 0;
    g_finish_armed       = false;
    g_finish_black_ticks = 0;
    g_lost_ticks         = 0;
    g_finish_reason      = FINISH_REASON_NONE;

    /* 重置灰度环历史量 */
    g_last_steer    = 0;
    g_last_pos_ctrl = 0;
    g_lost_cnt      = 0;

    if (g_mode == ROUTE_MODE_LINE_TEST) {
        g_state = ROUTE_STATE_FOLLOWING;
        Buzzer_Beep(POINT_BEEP_MS);
    } else if (g_mode == ROUTE_MODE_H_LAP) {
        g_state = ROUTE_STATE_LEAVE_START;
        Buzzer_Beep(POINT_BEEP_MS);
    } else if (g_mode == ROUTE_MODE_FINISH_TEST) {
        g_state = ROUTE_STATE_FOLLOWING;
        Buzzer_Beep(POINT_BEEP_MS);
    } else {
        g_state = ROUTE_STATE_STOPPED;
        Motor_Stop();
    }
}

void Route_Stop(void)
{
    Motor_Stop();
    g_state = ROUTE_STATE_STOPPED;
    Buzzer_Beep(POINT_BEEP_MS);
}

/* ── 模式设置 ── */

void Route_SetMode(RouteMode_t mode)
{
    if (mode == ROUTE_MODE_STOP || mode == ROUTE_MODE_LINE_TEST ||
        mode == ROUTE_MODE_H_LAP || mode == ROUTE_MODE_FINISH_TEST) {
        g_mode = mode;
    }
    /* 非法模式不赋值 (修复原 bug) */
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
    /* 由 main.c 在 ISR 中设置 g_run_start_ms */
    /* 这里返回从 start 到 now 的差值, 需要外部 ms_ticks */
    /* 暂时返回 0, 由 main.c 通过外部变量计算 */
    return 0;  /* 占位, 实际由 main.c 的 getter 提供 */
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
