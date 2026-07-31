/**
 * route_fsm.c — H 题专用状态机
 *
 * 整圈灰度巡线 + A 点横线识别 + 超程/超时保护
 * 所有参数均为运行时可调
 */

#include "route_fsm.h"
#include "buzzer.h"
#include "motor.h"

/* ── 默认值 ── */
#define DEFAULT_KP              1.80f
#define DEFAULT_KD              0.00f
#define DEFAULT_BASE_PWM        1200
#define DEFAULT_OUTPUT_LIM      1500
#define DEFAULT_TRIM            0
#define DEFAULT_DEAD_ZONE       3
#define DEFAULT_SLEW_STEP       75

#define DEFAULT_FMIN_CM         520.0f
#define DEFAULT_FMAX_CM         620.0f
#define DEFAULT_BLACK_MIN       4
#define DEFAULT_CONFIRM_COUNT   2
#define DEFAULT_BRAKE_MS        100

#define DEFAULT_LAP_MS          17500U
#define DEFAULT_TIME_WINDOW_MS  1000U
#define DEFAULT_SLOW_RATIO      0.70f
#define DEFAULT_SLOW_AHEAD_MS   1500U
#define DEFAULT_FALLBACK_MS     1200U
#define DEFAULT_TIMEOUT_MS      20000U

#define DEFAULT_LOST_MS         500U
#define DEFAULT_START_CLEAR_CM  25.0f

/* Q4 默认值 */
#define DEFAULT_Q4_PWM          1000
#define DEFAULT_Q4_RAMP_MS      800U
#define DEFAULT_Q4_ARM_CM       120.0f
#define DEFAULT_Q4_B_CM         145.0f
#define DEFAULT_Q4_YAW_DEG      4.0f
#define DEFAULT_Q4_POST_CM      25.0f
#define DEFAULT_Q4_STOP_MS      800U

/* Q5 默认值 */
#define DEFAULT_Q5_RAMP_MS      1000U
#define DEFAULT_Q5_OFFSET_CM    8.0f
#define DEFAULT_Q5_POST_CM      20.0f
#define DEFAULT_Q5_STOP_MS      1000U
#define DEFAULT_Q5_TIMEOUT_MS   29000U

#define POINT_BEEP_MS           200U

/* ── 状态枚举 ── */
typedef enum {
    ROUTE_STATE_STOPPED = 0,
    ROUTE_STATE_LEAVE_START,
    ROUTE_STATE_FOLLOWING,
    ROUTE_STATE_BRAKING,
    ROUTE_STATE_FINISHED,
    ROUTE_STATE_FAULT,
    /* Q4 专用状态 */
    Q4_STATE_RAMP,
    Q4_STATE_AB_RUN,
    Q4_STATE_POST,
    Q4_STATE_BRAKE,
    Q4_STATE_DONE,
    /* Q5 专用状态 */
    Q5_STATE_RAMP,
    Q5_STATE_LEAVE_A,
    Q5_STATE_FOLLOW,
    Q5_STATE_CROSS_WAIT,
    Q5_STATE_POST_A,
    Q5_STATE_BRAKE,
    Q5_STATE_DONE
} RouteState_t;

/* ── 请求标志 ── */
volatile bool g_route_start_request = false;
volatile bool g_route_stop_request  = false;
static volatile bool g_start_pending = false;

/* ── 全局状态 ── */
static RouteMode_t  g_mode;
static RouteState_t g_state;

/* ── 计时 ── */
static uint32_t g_run_start_ms;
static uint32_t g_run_elapsed_ms;
static uint32_t g_run_finish_ms;

/* ── 巡线参数（运行时可调） ── */
static volatile float   g_KP;
static volatile float   g_KD;
static volatile int16_t g_BASE_PWM;
static volatile int16_t g_OUTPUT_LIM;
static volatile int16_t g_TRIM;
static volatile int16_t g_dead_zone;
static volatile int16_t g_slew_step;

/* ── 停车参数（运行时可调） ── */
static volatile float    g_finish_min_cm;
static volatile float    g_finish_max_cm;
static volatile uint8_t  g_black_min;
static volatile uint8_t  g_finish_confirm_ticks;
static volatile uint16_t g_brake_duration_ticks;  /* 5ms tick */
static volatile uint16_t g_lost_max_ticks;        /* 5ms tick */
static volatile float    g_start_clear_cm;

/* ── 时间参数（运行时可调） ── */
static volatile uint32_t g_lap_ms;
static volatile uint32_t g_time_window_ms;
static volatile float    g_slow_ratio;
static volatile uint32_t g_slow_ahead_ms;
static volatile uint32_t g_fallback_ahead_ms;
static volatile uint32_t g_timeout_ms;

/* ── 计算得出的时间窗口 ── */
static uint32_t g_slowdown_ms;
static uint32_t g_finish_time_min_ms;
static uint32_t g_finish_time_max_ms;
static uint32_t g_fallback_stop_ms;

/* ── Q4 参数（运行时可调） ── */
static volatile int16_t  g_q4_pwm;
static volatile uint16_t g_q4_ramp_ms;
static volatile float    g_q4_arm_cm;
static volatile float    g_q4_b_cm;
static volatile float    g_q4_yaw_deg;
static volatile float    g_q4_post_cm;
static volatile uint16_t g_q4_stop_ms;

/* ── Q4 状态 ── */
static uint32_t g_q4_start_ms;
static float    g_q4_start_yaw;
static float    g_q4_b_dist;
static uint32_t g_q4_ab_time_ms;
static uint16_t g_q4_b_confirm_ticks;

/* ── Q5 参数（运行时可调） ── */
static volatile uint16_t g_q5_ramp_ms;
static volatile float    g_q5_offset_cm;
static volatile float    g_q5_post_cm;
static volatile uint16_t g_q5_stop_ms;
static volatile uint32_t g_q5_timeout_ms;

/* ── Q5 状态 ── */
static uint32_t g_q5_start_ms;
static float    g_q5_a_detect_dist;
static uint32_t g_q5_lap_time_ms;
static bool     g_q5_passed_a;
static uint16_t g_q5_ramp_ticks;
static uint16_t g_q5_brake_ticks;

/* ── 终点检测状态 ── */
static bool     g_finish_armed;
static uint8_t  g_finish_black_ticks;
static uint16_t g_lost_ticks;
static uint16_t g_brake_ticks;
static RouteFinishReason_t g_finish_reason;

/* ── PEAK 记录 ── */
static uint8_t  g_peak_bc;
static uint8_t  g_peak_raw;
static uint32_t g_peak_ms;
static float    g_peak_dist;

/* ── 巡线内部状态 ── */
static const int16_t WEIGHT[8] = { 200, 140, 75, 40, -40, -75, -140, -200 };
static int16_t  g_last_steer;
static int16_t  g_last_pos_ctrl;
static uint32_t g_lost_cnt;
static volatile int16_t g_line_error;
static volatile int16_t g_line_steer;

/* ── 工具 ── */
static int16_t ClampPwm(int32_t v)
{
    if (v > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (v < 0) return 0;
    return (int16_t)v;
}

static uint8_t CountBlack(uint8_t raw)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < 8; i++) count += (raw >> i) & 1U;
    return count;
}

static void RecalcTimeWindows(void)
{
    g_slowdown_ms        = (g_lap_ms > g_slow_ahead_ms) ? g_lap_ms - g_slow_ahead_ms : 0;
    g_finish_time_min_ms = (g_lap_ms > g_time_window_ms) ? g_lap_ms - g_time_window_ms : 0;
    g_finish_time_max_ms = g_lap_ms + g_time_window_ms;
    g_fallback_stop_ms   = g_lap_ms + g_fallback_ahead_ms;
}

/* ── 灰度巡线核心 ── */
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
        int16_t pos_ctrl = pos;
        if (pos_ctrl > -g_dead_zone && pos_ctrl < g_dead_zone) pos_ctrl = 0;

        int16_t d_pos = pos_ctrl - g_last_pos_ctrl;
        steer = (int16_t)(-((float)pos_ctrl * g_KP + (float)d_pos * g_KD));
        g_last_pos_ctrl = pos_ctrl;
        g_line_error = pos_ctrl;

        if (steer >  lim) steer =  lim;
        if (steer < -lim) steer = -lim;

        int16_t delta = steer - g_last_steer;
        if (delta >  g_slew_step) steer = g_last_steer + g_slew_step;
        if (delta < -g_slew_step) steer = g_last_steer - g_slew_step;

        g_last_steer = steer;
        g_lost_cnt = 0;
    } else {
        g_last_pos_ctrl = 0;
        steer = g_last_steer;
        g_line_error = 0;
        if (++g_lost_cnt > g_lost_max_ticks) {
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

/* ================================================================
 *  5ms 主循环
 * ================================================================ */
bool Route_Update5ms(uint32_t now_ms, float yaw_deg, uint8_t gray_raw)
{
    (void)yaw_deg;
    Buzzer_Update5ms();

    /* ── 处理 START/STOP 请求 ── */
    if (g_route_stop_request) {
        g_route_stop_request = false;
        g_start_pending = false;
        Route_Stop();
    } else if (g_route_start_request || g_start_pending) {
        g_route_start_request = false;
        g_start_pending = false;
        Motor_Stop();
        ResetRunState(now_ms);
        if (g_mode == ROUTE_MODE_H_LAP) {
            g_state = ROUTE_STATE_LEAVE_START;
            Buzzer_Beep(POINT_BEEP_MS);
        } else if (g_mode == ROUTE_MODE_LINE_TEST ||
                   g_mode == ROUTE_MODE_STRAIGHT) {
            g_state = ROUTE_STATE_FOLLOWING;
            Buzzer_Beep(POINT_BEEP_MS);
        } else if (g_mode == ROUTE_MODE_FINISH_TEST) {
            g_state = ROUTE_STATE_FOLLOWING;
            g_finish_armed = true;  /* 立即允许检测A线 */
            Buzzer_Beep(POINT_BEEP_MS);
        } else if (g_mode == ROUTE_MODE_Q4_AB) {
            g_state = Q4_STATE_RAMP;
            g_q4_start_ms = now_ms;
            g_q4_start_yaw = yaw_deg;
            g_q4_ab_time_ms = 0;
            g_q4_b_confirm_ticks = 0;
            g_q4_b_dist = 0;
            Buzzer_Beep(POINT_BEEP_MS);
        } else if (g_mode == ROUTE_MODE_Q5_LAP) {
            g_state = Q5_STATE_RAMP;
            g_q5_start_ms = now_ms;
            g_q5_a_detect_dist = 0;
            g_q5_lap_time_ms = 0;
            g_q5_passed_a = false;
            g_q5_ramp_ticks = 0;
            g_q5_brake_ticks = 0;
            g_finish_armed = false;
            g_finish_black_ticks = 0;
            Buzzer_Beep(POINT_BEEP_MS);
        } else {
            g_state = ROUTE_STATE_STOPPED;
        }
    }

    switch (g_state) {

    case ROUTE_STATE_STOPPED:
        Motor_Stop();
        return false;

    /* ── 离开起点 ── */
    case ROUTE_STATE_LEAVE_START:
    {
        LineFollow_Update(gray_raw);
        float dist = Encoder_GetAverageDistanceCm();
        if (dist >= g_start_clear_cm) {
            g_finish_armed = true;
            g_state = ROUTE_STATE_FOLLOWING;
        }
        g_run_elapsed_ms = now_ms - g_run_start_ms;
        return true;
    }

    /* ── 巡线 + 终点检测 ── */
    case ROUTE_STATE_FOLLOWING:
    {
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_run_start_ms;
        g_run_elapsed_ms = elapsed;

        /* 直走模式 */
        if (g_mode == ROUTE_MODE_STRAIGHT) {
            Motor_SetLeftSpeed(ClampPwm((int32_t)g_BASE_PWM + g_TRIM));
            Motor_SetRightSpeed(ClampPwm((int32_t)g_BASE_PWM - g_TRIM));
            g_line_error = 0;
            g_line_steer = 0;
            /* 3秒超时保护 */
            if (elapsed >= 3000U) {
                g_run_finish_ms = elapsed;
                g_run_elapsed_ms = elapsed;
                g_state = ROUTE_STATE_FINISHED;
                Motor_Stop();
            }
            return true;
        }

        /* 终点前降速 */
        if (elapsed >= g_slowdown_ms && g_finish_armed) {
            int16_t slow_base = (int16_t)((float)g_BASE_PWM * g_slow_ratio);
            int8_t s[8];
            for (uint8_t i = 0; i < 8; i++) s[i] = (gray_raw >> i) & 1;
            int16_t pos = 0;
            for (uint8_t i = 0; i < 8; i++) pos += WEIGHT[i] * s[i];
            bool has_line = (s[0]||s[1]||s[2]||s[3]||s[4]||s[5]||s[6]||s[7]);
            bool all_black = (gray_raw == 0xFF);
            int16_t steer = 0;
            if (has_line && !all_black) {
                int16_t pos_ctrl = pos;
                if (pos_ctrl > -g_dead_zone && pos_ctrl < g_dead_zone) pos_ctrl = 0;
                int16_t d_pos = pos_ctrl - g_last_pos_ctrl;
                steer = (int16_t)(-((float)pos_ctrl * g_KP + (float)d_pos * g_KD));
                g_last_pos_ctrl = pos_ctrl;
                g_line_error = pos_ctrl;
                if (steer >  g_OUTPUT_LIM) steer =  g_OUTPUT_LIM;
                if (steer < -g_OUTPUT_LIM) steer = -g_OUTPUT_LIM;
                int16_t delta = steer - g_last_steer;
                if (delta >  g_slew_step) steer = g_last_steer + g_slew_step;
                if (delta < -g_slew_step) steer = g_last_steer - g_slew_step;
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
            LineFollow_Update(gray_raw);
        }

        /* A 横线检测 */
        bool in_time = (elapsed >= g_finish_time_min_ms &&
                        elapsed <= g_finish_time_max_ms);
        bool in_dist = (dist >= g_finish_min_cm &&
                        dist <= g_finish_max_cm);

        /* MODE 3: 绕过时间/距离窗口，直接检测 */
        bool finish_window = (g_mode == ROUTE_MODE_FINISH_TEST) ? true : (in_time && in_dist);

        /* PEAK 记录 */
        if (g_finish_armed && (in_time || g_mode == ROUTE_MODE_FINISH_TEST)) {
            uint8_t bc = CountBlack(gray_raw);
            if (bc > g_peak_bc) {
                g_peak_bc   = bc;
                g_peak_raw  = gray_raw;
                g_peak_ms   = elapsed;
                g_peak_dist = dist;
            }
        }

        if (g_finish_armed && finish_window) {
            bool candidate = (CountBlack(gray_raw) >= g_black_min);
            if (candidate) {
                if (g_finish_black_ticks < g_finish_confirm_ticks)
                    g_finish_black_ticks++;
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

        /* 横线失效后备停车 */
        if (g_finish_armed && elapsed >= g_fallback_stop_ms) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_brake_ticks = 0;
            g_state = ROUTE_STATE_BRAKING;
            Motor_Brake();
            return true;
        }

        /* 丢线保护 */
        if (gray_raw == 0x00) g_lost_ticks++;
        else g_lost_ticks = 0;
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed;
            g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT;
            Motor_Stop();
            return true;
        }

        /* 超程保护 */
        if (dist >= g_finish_max_cm + 100.0f) {
            g_finish_reason = FINISH_REASON_OVERRUN;
            g_brake_ticks = 0;
            g_state = ROUTE_STATE_FAULT;
            Motor_Brake();
            return true;
        }

        /* 安全超时 */
        if (elapsed >= g_timeout_ms) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_run_finish_ms = elapsed;
            g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT;
            Motor_Stop();
            return true;
        }

        return true;
    }

    /* ── 刹车 ── */
    case ROUTE_STATE_BRAKING:
        Motor_Brake();
        g_brake_ticks++;
        if (g_brake_ticks >= g_brake_duration_ticks) {
            g_run_finish_ms = now_ms - g_run_start_ms;
            g_run_elapsed_ms = g_run_finish_ms;
            Motor_Stop();
            g_state = ROUTE_STATE_FINISHED;
        }
        g_run_elapsed_ms = now_ms - g_run_start_ms;
        return true;

    case ROUTE_STATE_FINISHED:
        Motor_Stop();
        return true;

    case ROUTE_STATE_FAULT:
        if (g_finish_reason == FINISH_REASON_OVERRUN &&
            g_brake_ticks < g_brake_duration_ticks) {
            Motor_Brake();
            g_brake_ticks++;
            g_run_elapsed_ms = now_ms - g_run_start_ms;
        } else {
            if (g_run_finish_ms == 0) {
                g_run_finish_ms = now_ms - g_run_start_ms;
                g_run_elapsed_ms = g_run_finish_ms;
            }
            Motor_Stop();
        }
        return true;

    /* ── Q4: 软启动 ── */
    case Q4_STATE_RAMP:
    {
        uint32_t elapsed = now_ms - g_q4_start_ms;
        g_run_elapsed_ms = elapsed;
        int16_t pwm = (int16_t)((float)g_q4_pwm * (float)elapsed / (float)g_q4_ramp_ms);
        if (pwm > g_q4_pwm) pwm = g_q4_pwm;
        /* 巡线 */
        LineFollow_Update(gray_raw);
        /* 覆盖电机输出为 ramp pwm */
        Motor_SetLeftSpeed(ClampPwm((int32_t)pwm + g_line_steer + g_TRIM));
        Motor_SetRightSpeed(ClampPwm((int32_t)pwm - g_line_steer - g_TRIM));
        if (elapsed >= g_q4_ramp_ms) {
            g_state = Q4_STATE_AB_RUN;
        }
        /* 安全保护 */
        if (gray_raw == 0x00) { g_lost_ticks++; } else { g_lost_ticks = 0; }
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        return true;
    }

    /* ── Q4: A→B 巡线（用 Q4PWM） ── */
    case Q4_STATE_AB_RUN:
    {
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_q4_start_ms;
        g_run_elapsed_ms = elapsed;

        /* 用 Q4PWM 巡线，不用公共 BASE */
        int8_t s[8];
        for (uint8_t i = 0; i < 8; i++) s[i] = (gray_raw >> i) & 1;
        int16_t pos = 0;
        for (uint8_t i = 0; i < 8; i++) pos += WEIGHT[i] * s[i];
        bool has = (s[0]||s[1]||s[2]||s[3]||s[4]||s[5]||s[6]||s[7]);
        bool alb = (gray_raw == 0xFF);
        int16_t steer = 0;
        if (has && !alb) {
            int16_t pc = pos;
            if (pc > -g_dead_zone && pc < g_dead_zone) pc = 0;
            int16_t dp = pc - g_last_pos_ctrl;
            steer = (int16_t)(-((float)pc * g_KP + (float)dp * g_KD));
            g_last_pos_ctrl = pc; g_line_error = pc;
            if (steer > g_OUTPUT_LIM) steer = g_OUTPUT_LIM;
            if (steer < -g_OUTPUT_LIM) steer = -g_OUTPUT_LIM;
            int16_t d = steer - g_last_steer;
            if (d > g_slew_step) steer = g_last_steer + g_slew_step;
            if (d < -g_slew_step) steer = g_last_steer - g_slew_step;
            g_last_steer = steer; g_lost_cnt = 0;
        } else {
            g_last_pos_ctrl = 0; steer = g_last_steer; g_line_error = 0;
        }
        g_line_steer = steer;
        Motor_SetLeftSpeed(ClampPwm((int32_t)g_q4_pwm + steer + g_TRIM));
        Motor_SetRightSpeed(ClampPwm((int32_t)g_q4_pwm - steer - g_TRIM));

        /* 检测 B 点 */
        float yaw_diff = yaw_deg - g_q4_start_yaw;
        if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
        if (yaw_diff < -180.0f) yaw_diff += 360.0f;
        float yaw_abs = (yaw_diff < 0) ? -yaw_diff : yaw_diff;

        bool dist_ok = (dist >= g_q4_arm_cm && dist <= g_q4_b_cm + 30.0f);
        bool yaw_ok  = (yaw_abs >= g_q4_yaw_deg);

        if (dist_ok && yaw_ok) {
            if (g_q4_b_confirm_ticks < 4U) {
                g_q4_b_confirm_ticks++;
            } else {
                g_q4_ab_time_ms = elapsed;
                g_q4_b_dist = dist;
                g_run_finish_ms = elapsed;
                g_state = Q4_STATE_POST;
            }
        } else {
            g_q4_b_confirm_ticks = 0;
        }

        /* 安全保护 */
        if (gray_raw == 0x00) { g_lost_ticks++; } else { g_lost_ticks = 0; }
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        if (elapsed >= 10000U) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        if (dist >= 300.0f) {
            g_finish_reason = FINISH_REASON_OVERRUN;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Brake();
        }
        return true;
    }

    /* ── Q4: 过 B 后继续（用 Q4PWM 巡线） ── */
    case Q4_STATE_POST:
    {
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_q4_start_ms;
        g_run_elapsed_ms = elapsed;

        /* 继续用 Q4PWM 巡线，不用公共 BASE，避免速度跳变 */
        int8_t s[8];
        for (uint8_t i = 0; i < 8; i++) s[i] = (gray_raw >> i) & 1;
        int16_t pos = 0;
        for (uint8_t i = 0; i < 8; i++) pos += WEIGHT[i] * s[i];
        bool has = (s[0]||s[1]||s[2]||s[3]||s[4]||s[5]||s[6]||s[7]);
        bool alb = (gray_raw == 0xFF);
        int16_t steer = 0;
        if (has && !alb) {
            int16_t pc = pos;
            if (pc > -g_dead_zone && pc < g_dead_zone) pc = 0;
            int16_t dp = pc - g_last_pos_ctrl;
            steer = (int16_t)(-((float)pc * g_KP + (float)dp * g_KD));
            g_last_pos_ctrl = pc; g_line_error = pc;
            if (steer > g_OUTPUT_LIM) steer = g_OUTPUT_LIM;
            if (steer < -g_OUTPUT_LIM) steer = -g_OUTPUT_LIM;
            int16_t d = steer - g_last_steer;
            if (d > g_slew_step) steer = g_last_steer + g_slew_step;
            if (d < -g_slew_step) steer = g_last_steer - g_slew_step;
            g_last_steer = steer; g_lost_cnt = 0;
        } else {
            g_last_pos_ctrl = 0; steer = g_last_steer; g_line_error = 0;
        }
        g_line_steer = steer;
        Motor_SetLeftSpeed(ClampPwm((int32_t)g_q4_pwm + steer + g_TRIM));
        Motor_SetRightSpeed(ClampPwm((int32_t)g_q4_pwm - steer - g_TRIM));

        if (dist - g_q4_b_dist >= g_q4_post_cm) {
            g_state = Q4_STATE_BRAKE;
            g_brake_ticks = 0;
        }

        /* 安全保护 */
        if (gray_raw == 0x00) { g_lost_ticks++; } else { g_lost_ticks = 0; }
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        if (elapsed >= 10000U) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        return true;
    }

    /* ── Q4: 软减速停车 ── */
    case Q4_STATE_BRAKE:
    {
        uint32_t brake_elapsed = (now_ms - g_q4_start_ms) - g_q4_ab_time_ms - (uint32_t)(g_q4_post_cm / 30.0f * 1000.0f);
        /* 简化：用 brake_ticks 计数 */
        g_brake_ticks++;
        g_run_elapsed_ms = now_ms - g_q4_start_ms;

        int16_t pwm = g_q4_pwm - (int16_t)((float)g_q4_pwm * (float)g_brake_ticks / (float)(g_q4_stop_ms / 5U));
        if (pwm < 0) pwm = 0;

        /* 低速时直接停车 */
        if (pwm <= 50 || g_brake_ticks >= g_q4_stop_ms / 5U) {
            Motor_Stop();
            g_state = Q4_STATE_DONE;
            g_run_finish_ms = now_ms - g_q4_start_ms;
            g_run_elapsed_ms = g_run_finish_ms;
        } else {
            Motor_SetLeftSpeed(ClampPwm((int32_t)pwm + g_line_steer + g_TRIM));
            Motor_SetRightSpeed(ClampPwm((int32_t)pwm - g_line_steer - g_TRIM));
        }
        return true;
    }

    /* ── Q4: 完成 ── */
    case Q4_STATE_DONE:
        Motor_Stop();
        return true;

    /* ── Q5: 软启动 ── */
    case Q5_STATE_RAMP:
    {
        g_q5_ramp_ticks++;
        uint32_t elapsed = now_ms - g_q5_start_ms;
        g_run_elapsed_ms = elapsed;
        int16_t pwm = (int16_t)((float)g_BASE_PWM *
            (float)g_q5_ramp_ticks / (float)(g_q5_ramp_ms / 5U));
        if (pwm > g_BASE_PWM) pwm = g_BASE_PWM;
        /* 用当前 pwm 作为 base 巡线 */
        int8_t s[8];
        for (uint8_t i = 0; i < 8; i++) s[i] = (gray_raw >> i) & 1;
        int16_t pos = 0;
        for (uint8_t i = 0; i < 8; i++) pos += WEIGHT[i] * s[i];
        bool has = (s[0]||s[1]||s[2]||s[3]||s[4]||s[5]||s[6]||s[7]);
        bool alb = (gray_raw == 0xFF);
        int16_t steer = 0;
        if (has && !alb) {
            int16_t pc = pos;
            if (pc > -g_dead_zone && pc < g_dead_zone) pc = 0;
            int16_t dp = pc - g_last_pos_ctrl;
            steer = (int16_t)(-((float)pc * g_KP + (float)dp * g_KD));
            g_last_pos_ctrl = pc; g_line_error = pc;
            if (steer > g_OUTPUT_LIM) steer = g_OUTPUT_LIM;
            if (steer < -g_OUTPUT_LIM) steer = -g_OUTPUT_LIM;
            int16_t d = steer - g_last_steer;
            if (d > g_slew_step) steer = g_last_steer + g_slew_step;
            if (d < -g_slew_step) steer = g_last_steer - g_slew_step;
            g_last_steer = steer; g_lost_cnt = 0;
        } else {
            g_last_pos_ctrl = 0; steer = g_last_steer; g_line_error = 0;
        }
        g_line_steer = steer;
        Motor_SetLeftSpeed(ClampPwm((int32_t)pwm + steer + g_TRIM));
        Motor_SetRightSpeed(ClampPwm((int32_t)pwm - steer - g_TRIM));
        if (g_q5_ramp_ticks >= g_q5_ramp_ms / 5U) {
            g_finish_armed = false;
            g_state = Q5_STATE_LEAVE_A;
        }
        return true;
    }

    /* ── Q5: 离开起点 ── */
    case Q5_STATE_LEAVE_A:
    {
        LineFollow_Update(gray_raw);
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_q5_start_ms;
        g_run_elapsed_ms = elapsed;
        if (dist >= g_start_clear_cm) {
            g_finish_armed = true;
            g_state = Q5_STATE_FOLLOW;
        }
        return true;
    }

    /* ── Q5: 一圈巡线 ── */
    case Q5_STATE_FOLLOW:
    {
        LineFollow_Update(gray_raw);
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_q5_start_ms;
        g_run_elapsed_ms = elapsed;

        /* A 线检测窗口 */
        bool in_time = (elapsed >= g_finish_time_min_ms &&
                        elapsed <= g_finish_time_max_ms);
        bool in_dist = (dist >= g_finish_min_cm &&
                        dist <= g_finish_max_cm);

        /* PEAK 记录 */
        if (g_finish_armed && in_time) {
            uint8_t bc = CountBlack(gray_raw);
            if (bc > g_peak_bc) {
                g_peak_bc = bc; g_peak_raw = gray_raw;
                g_peak_ms = elapsed; g_peak_dist = dist;
            }
        }

        if (g_finish_armed && in_time && in_dist) {
            bool candidate = (CountBlack(gray_raw) >= g_black_min);
            if (candidate) {
                if (g_finish_black_ticks < g_finish_confirm_ticks)
                    g_finish_black_ticks++;
            } else {
                g_finish_black_ticks = 0;
            }
            if (g_finish_black_ticks >= g_finish_confirm_ticks) {
                g_q5_a_detect_dist = dist;
                g_state = Q5_STATE_CROSS_WAIT;
            }
        }

        /* 丢线保护 */
        if (gray_raw == 0x00) g_lost_ticks++;
        else g_lost_ticks = 0;
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }

        /* 超程 */
        if (dist >= g_finish_max_cm + 100.0f) {
            g_finish_reason = FINISH_REASON_OVERRUN;
            g_brake_ticks = 0;
            g_state = ROUTE_STATE_FAULT; Motor_Brake();
        }

        /* 超时 */
        if (elapsed >= g_q5_timeout_ms) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }

        return true;
    }

    /* ── Q5: 等待测试点通过A ── */
    case Q5_STATE_CROSS_WAIT:
    {
        LineFollow_Update(gray_raw);
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_q5_start_ms;
        g_run_elapsed_ms = elapsed;

        if (dist >= g_q5_a_detect_dist + g_q5_offset_cm) {
            g_q5_lap_time_ms = elapsed;
            g_q5_passed_a = true;
            g_run_finish_ms = elapsed;
            g_finish_reason = FINISH_REASON_LINE; /* 标记成功 */
            g_state = Q5_STATE_POST_A;
        }
        /* 安全保护 */
        if (elapsed >= g_q5_timeout_ms) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        if (gray_raw == 0x00) { g_lost_ticks++; } else { g_lost_ticks = 0; }
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        if (dist >= g_finish_max_cm + 100.0f) {
            g_finish_reason = FINISH_REASON_OVERRUN;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Brake();
        }
        return true;
    }

    /* ── Q5: 通过A后继续 ── */
    case Q5_STATE_POST_A:
    {
        LineFollow_Update(gray_raw);
        float dist = Encoder_GetAverageDistanceCm();
        uint32_t elapsed = now_ms - g_q5_start_ms;
        g_run_elapsed_ms = g_q5_lap_time_ms; /* 显示冻结的圈时 */

        if (dist >= g_q5_a_detect_dist + g_q5_offset_cm + g_q5_post_cm) {
            g_q5_brake_ticks = 0;
            g_state = Q5_STATE_BRAKE;
        }
        /* 安全保护 */
        if (elapsed >= g_q5_timeout_ms) {
            g_finish_reason = FINISH_REASON_TIMEOUT;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        if (gray_raw == 0x00) { g_lost_ticks++; } else { g_lost_ticks = 0; }
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        if (dist >= g_finish_max_cm + 100.0f) {
            g_finish_reason = FINISH_REASON_OVERRUN;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Brake();
        }
        if (g_lost_ticks >= g_lost_max_ticks) {
            g_finish_reason = FINISH_REASON_LOST;
            g_run_finish_ms = elapsed; g_run_elapsed_ms = elapsed;
            g_state = ROUTE_STATE_FAULT; Motor_Stop();
        }
        return true;
    }

    /* ── Q5: 平滑减速 ── */
    case Q5_STATE_BRAKE:
    {
        g_q5_brake_ticks++;
        g_run_elapsed_ms = g_q5_lap_time_ms;

        int16_t pwm = g_BASE_PWM - (int16_t)((float)g_BASE_PWM *
            (float)g_q5_brake_ticks / (float)(g_q5_stop_ms / 5U));
        if (pwm < 0) pwm = 0;

        if (pwm <= 50 || g_q5_brake_ticks >= g_q5_stop_ms / 5U) {
            Motor_Stop();
            g_state = Q5_STATE_DONE;
        } else {
            Motor_SetLeftSpeed(ClampPwm((int32_t)pwm + g_TRIM));
            Motor_SetRightSpeed(ClampPwm((int32_t)pwm - g_TRIM));
        }
        return true;
    }

    /* ── Q5: 完成 ── */
    case Q5_STATE_DONE:
        Motor_Stop();
        g_run_elapsed_ms = g_q5_lap_time_ms;
        return true;

    default:
        Motor_Stop();
        g_state = ROUTE_STATE_STOPPED;
        return false;
    }
}

/* ================================================================
 *  生命周期
 * ================================================================ */
void Route_Init(void)
{
    g_mode  = ROUTE_MODE_H_LAP;
    g_state = ROUTE_STATE_STOPPED;

    g_run_start_ms = 0; g_run_elapsed_ms = 0; g_run_finish_ms = 0;

    g_KP         = DEFAULT_KP;
    g_KD         = DEFAULT_KD;
    g_BASE_PWM   = DEFAULT_BASE_PWM;
    g_OUTPUT_LIM = DEFAULT_OUTPUT_LIM;
    g_TRIM       = DEFAULT_TRIM;
    g_dead_zone  = DEFAULT_DEAD_ZONE;
    g_slew_step  = DEFAULT_SLEW_STEP;

    g_finish_min_cm       = DEFAULT_FMIN_CM;
    g_finish_max_cm       = DEFAULT_FMAX_CM;
    g_black_min           = DEFAULT_BLACK_MIN;
    g_finish_confirm_ticks = DEFAULT_CONFIRM_COUNT;
    g_brake_duration_ticks = DEFAULT_BRAKE_MS / 5U;
    g_lost_max_ticks       = DEFAULT_LOST_MS / 5U;
    g_start_clear_cm       = DEFAULT_START_CLEAR_CM;

    g_lap_ms            = DEFAULT_LAP_MS;
    g_time_window_ms    = DEFAULT_TIME_WINDOW_MS;
    g_slow_ratio        = DEFAULT_SLOW_RATIO;
    g_slow_ahead_ms     = DEFAULT_SLOW_AHEAD_MS;
    g_fallback_ahead_ms = DEFAULT_FALLBACK_MS;
    g_timeout_ms        = DEFAULT_TIMEOUT_MS;

    RecalcTimeWindows();
    g_timeout_ms = g_fallback_stop_ms + 1300U;

    g_finish_armed = false; g_finish_black_ticks = 0;
    g_lost_ticks = 0; g_brake_ticks = 0;
    g_finish_reason = FINISH_REASON_NONE;
    g_start_pending = false;

    g_last_steer = 0; g_last_pos_ctrl = 0; g_lost_cnt = 0;
    g_line_error = 0; g_line_steer = 0;
    g_peak_bc = 0; g_peak_raw = 0; g_peak_ms = 0; g_peak_dist = 0.0f;

    g_q4_pwm     = DEFAULT_Q4_PWM;
    g_q4_ramp_ms = DEFAULT_Q4_RAMP_MS;
    g_q4_arm_cm  = DEFAULT_Q4_ARM_CM;
    g_q4_b_cm    = DEFAULT_Q4_B_CM;
    g_q4_yaw_deg = DEFAULT_Q4_YAW_DEG;
    g_q4_post_cm = DEFAULT_Q4_POST_CM;
    g_q4_stop_ms = DEFAULT_Q4_STOP_MS;
    g_q4_start_ms = 0; g_q4_ab_time_ms = 0;
    g_q4_start_yaw = 0; g_q4_b_dist = 0; g_q4_b_confirm_ticks = 0;

    g_q5_ramp_ms    = DEFAULT_Q5_RAMP_MS;
    g_q5_offset_cm  = DEFAULT_Q5_OFFSET_CM;
    g_q5_post_cm    = DEFAULT_Q5_POST_CM;
    g_q5_stop_ms    = DEFAULT_Q5_STOP_MS;
    g_q5_timeout_ms = DEFAULT_Q5_TIMEOUT_MS;
    g_q5_start_ms = 0; g_q5_a_detect_dist = 0;
    g_q5_lap_time_ms = 0; g_q5_passed_a = false;
    g_q5_ramp_ticks = 0; g_q5_brake_ticks = 0;
}

void Route_Start(void) { g_start_pending = true; }

void Route_Stop(void)
{
    Motor_Stop();
    g_state = ROUTE_STATE_STOPPED;
    g_start_pending = false;
    g_brake_ticks = 0;
    Buzzer_Beep(POINT_BEEP_MS);
}

/* ================================================================
 *  模式
 * ================================================================ */
void Route_SetMode(RouteMode_t mode)
{
    if ((int)mode >= 0 && (int)mode <= (int)ROUTE_MODE_Q5_LAP) g_mode = mode;
}
RouteMode_t Route_GetMode(void) { return g_mode; }

/* ================================================================
 *  巡线参数 Setter/Getter
 * ================================================================ */
void Route_SetKp(float v) { if (v < 0) v = 0; if (v > 10) v = 10; g_KP = v; }
void Route_SetKd(float v) { if (v < 0) v = 0; if (v > 10) v = 10; g_KD = v; }
void Route_SetBasePwm(int16_t v) { if (v < 0) v = 0; if (v > MOTOR_PWM_MAX) v = MOTOR_PWM_MAX; g_BASE_PWM = v; }
void Route_SetOutputLim(int16_t v) { if (v < 0) v = 0; if (v > MOTOR_PWM_MAX) v = MOTOR_PWM_MAX; g_OUTPUT_LIM = v; }
void Route_SetTrim(int16_t v) { if (v < -500) v = -500; if (v > 500) v = 500; g_TRIM = v; }
void Route_SetDeadZone(int16_t v) { if (v < 0) v = 0; if (v > 50) v = 50; g_dead_zone = v; }
void Route_SetSlewStep(int16_t v) { if (v < 1) v = 1; if (v > 1000) v = 1000; g_slew_step = v; }

float   Route_GetKp(void)        { return g_KP; }
float   Route_GetKd(void)        { return g_KD; }
int16_t Route_GetBasePwm(void)   { return g_BASE_PWM; }
int16_t Route_GetOutputLim(void) { return g_OUTPUT_LIM; }
int16_t Route_GetTrim(void)      { return g_TRIM; }
int16_t Route_GetDeadZone(void)  { return g_dead_zone; }
int16_t Route_GetSlewStep(void)  { return g_slew_step; }

/* ================================================================
 *  停车参数
 * ================================================================ */
void Route_SetFinishMinDist(float v) { if (v < 100) v = 100; if (v > g_finish_max_cm - 50) v = g_finish_max_cm - 50; g_finish_min_cm = v; }
void Route_SetFinishMaxDist(float v) { if (v < g_finish_min_cm + 50) v = g_finish_min_cm + 50; if (v > 1000) v = 1000; g_finish_max_cm = v; }
void Route_SetBlackMin(uint8_t v) { if (v < 1) v = 1; if (v > 8) v = 8; g_black_min = v; }
void Route_SetFinishConfirmTicks(uint8_t v) { if (v < 1) v = 1; if (v > 20) v = 20; g_finish_confirm_ticks = v; }
void Route_SetBrakeDurationMs(uint16_t v) { if (v < 5) v = 5; if (v > 500) v = 500; g_brake_duration_ticks = v / 5U; }
void Route_SetLostMs(uint16_t v) { if (v < 100) v = 100; if (v > 5000) v = 5000; g_lost_max_ticks = v / 5U; }
void Route_SetStartClearCm(float v) { if (v < 5) v = 5; if (v > 100) v = 100; g_start_clear_cm = v; }

float    Route_GetFinishMinDist(void)       { return g_finish_min_cm; }
float    Route_GetFinishMaxDist(void)       { return g_finish_max_cm; }
uint8_t  Route_GetBlackMin(void)            { return g_black_min; }
uint8_t  Route_GetFinishConfirmTicks(void)  { return g_finish_confirm_ticks; }
uint16_t Route_GetBrakeDurationMs(void)     { return g_brake_duration_ticks * 5U; }
uint16_t Route_GetLostMs(void)              { return g_lost_max_ticks * 5U; }
float    Route_GetStartClearCm(void)        { return g_start_clear_cm; }

/* ================================================================
 *  时间参数
 * ================================================================ */
void Route_SetLapTargetMs(uint32_t v)     { if (v < 5000) v = 5000; if (v > 30000) v = 30000; g_lap_ms = v; RecalcTimeWindows(); g_timeout_ms = g_fallback_stop_ms + 1300U; }
void Route_SetTimeWindowMs(uint32_t v)    { if (v < 100) v = 100; if (v > 3000) v = 3000; g_time_window_ms = v; RecalcTimeWindows(); }
void Route_SetSlowRatio(float v)          { if (v < 0.2f) v = 0.2f; if (v > 1.0f) v = 1.0f; g_slow_ratio = v; }
void Route_SetSlowAheadMs(uint32_t v)     { if (v > 5000) v = 5000; g_slow_ahead_ms = v; RecalcTimeWindows(); }
void Route_SetFallbackAheadMs(uint32_t v) { if (v < 200) v = 200; if (v > 5000) v = 5000; g_fallback_ahead_ms = v; RecalcTimeWindows(); g_timeout_ms = g_fallback_stop_ms + 1300U; }
void Route_SetTimeoutMs(uint32_t v)       { if (v < 5000) v = 5000; if (v > 60000) v = 60000; g_timeout_ms = v; }

uint32_t Route_GetLapTargetMs(void)       { return g_lap_ms; }
uint32_t Route_GetTimeWindowMs(void)      { return g_time_window_ms; }
float    Route_GetSlowRatio(void)         { return g_slow_ratio; }
uint32_t Route_GetSlowAheadMs(void)       { return g_slow_ahead_ms; }
uint32_t Route_GetFallbackAheadMs(void)   { return g_fallback_ahead_ms; }
uint32_t Route_GetTimeoutMs(void)         { return g_timeout_ms; }

/* ================================================================
 *  状态查询
 * ================================================================ */
bool Route_IsActive(void)
{
    return (g_state == ROUTE_STATE_LEAVE_START ||
            g_state == ROUTE_STATE_FOLLOWING ||
            g_state == ROUTE_STATE_BRAKING ||
            g_state == Q4_STATE_RAMP ||
            g_state == Q4_STATE_AB_RUN ||
            g_state == Q4_STATE_POST ||
            g_state == Q4_STATE_BRAKE ||
            g_state == Q5_STATE_RAMP ||
            g_state == Q5_STATE_LEAVE_A ||
            g_state == Q5_STATE_FOLLOW ||
            g_state == Q5_STATE_CROSS_WAIT ||
            g_state == Q5_STATE_POST_A ||
            g_state == Q5_STATE_BRAKE);
}

bool Route_IsFinished(void)
{
    return (g_state == ROUTE_STATE_FINISHED || g_state == ROUTE_STATE_FAULT ||
            g_state == Q4_STATE_DONE || g_state == Q5_STATE_DONE);
}

uint32_t Route_GetElapsedMs(void)  { return g_run_elapsed_ms; }
float    Route_GetDistanceCm(void) { return Encoder_GetAverageDistanceCm(); }
RouteFinishReason_t Route_GetFinishReason(void) { return g_finish_reason; }

const char *Route_GetStateName(void)
{
    switch (g_state) {
    case ROUTE_STATE_STOPPED:     return "STOP";
    case ROUTE_STATE_LEAVE_START: return "LVST";
    case ROUTE_STATE_FOLLOWING:   return "FOLL";
    case ROUTE_STATE_BRAKING:     return "BRK ";
    case ROUTE_STATE_FINISHED:    return "DONE";
    case ROUTE_STATE_FAULT:       return "ERR ";
    case Q4_STATE_RAMP:           return "RAMP";
    case Q4_STATE_AB_RUN:         return "AB  ";
    case Q4_STATE_POST:           return "POST";
    case Q4_STATE_BRAKE:          return "QBK ";
    case Q4_STATE_DONE:           return "QDNE";
    case Q5_STATE_RAMP:           return "5RMP";
    case Q5_STATE_LEAVE_A:        return "5LVA";
    case Q5_STATE_FOLLOW:         return "5FOL";
    case Q5_STATE_CROSS_WAIT:     return "5XWT";
    case Q5_STATE_POST_A:         return "5PST";
    case Q5_STATE_BRAKE:          return "5BRK";
    case Q5_STATE_DONE:           return "5DNE";
    default:                      return "????";
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

/* ================================================================
 *  Q4 参数
 * ================================================================ */
void  Route_SetQ4Pwm(int16_t v)      { if (v < 0) v = 0; if (v > MOTOR_PWM_MAX) v = MOTOR_PWM_MAX; g_q4_pwm = v; }
void  Route_SetQ4RampMs(uint16_t v)  { if (v < 100) v = 100; if (v > 3000) v = 3000; g_q4_ramp_ms = v; }
void  Route_SetQ4ArmCm(float v)      { if (v < 50) v = 50; if (v > 300) v = 300; g_q4_arm_cm = v; }
void  Route_SetQ4BCm(float v)        { if (v < 50) v = 50; if (v > 300) v = 300; g_q4_b_cm = v; }
void  Route_SetQ4YawDeg(float v)     { if (v < 1) v = 1; if (v > 90) v = 90; g_q4_yaw_deg = v; }
void  Route_SetQ4PostCm(float v)     { if (v < 0) v = 0; if (v > 100) v = 100; g_q4_post_cm = v; }
void  Route_SetQ4StopMs(uint16_t v)  { if (v < 100) v = 100; if (v > 3000) v = 3000; g_q4_stop_ms = v; }

int16_t  Route_GetQ4Pwm(void)     { return g_q4_pwm; }
uint16_t Route_GetQ4RampMs(void)  { return g_q4_ramp_ms; }
float    Route_GetQ4ArmCm(void)   { return g_q4_arm_cm; }
float    Route_GetQ4BCm(void)     { return g_q4_b_cm; }
float    Route_GetQ4YawDeg(void)  { return g_q4_yaw_deg; }
float    Route_GetQ4PostCm(void)  { return g_q4_post_cm; }
uint16_t Route_GetQ4StopMs(void)  { return g_q4_stop_ms; }
uint32_t Route_GetQ4AbTimeMs(void) { return g_q4_ab_time_ms; }

/* ================================================================
 *  Q5 参数
 * ================================================================ */
void  Route_SetQ5RampMs(uint16_t v)   { if (v < 100) v = 100; if (v > 3000) v = 3000; g_q5_ramp_ms = v; }
void  Route_SetQ5OffsetCm(float v)    { if (v < 0) v = 0; if (v > 30) v = 30; g_q5_offset_cm = v; }
void  Route_SetQ5PostCm(float v)      { if (v < 0) v = 0; if (v > 100) v = 100; g_q5_post_cm = v; }
void  Route_SetQ5StopMs(uint16_t v)   { if (v < 100) v = 100; if (v > 3000) v = 3000; g_q5_stop_ms = v; }
void  Route_SetQ5TimeoutMs(uint32_t v) { if (v < 10000) v = 10000; if (v > 60000) v = 60000; g_q5_timeout_ms = v; }

uint16_t Route_GetQ5RampMs(void)      { return g_q5_ramp_ms; }
float    Route_GetQ5OffsetCm(void)    { return g_q5_offset_cm; }
float    Route_GetQ5PostCm(void)      { return g_q5_post_cm; }
uint16_t Route_GetQ5StopMs(void)      { return g_q5_stop_ms; }
uint32_t Route_GetQ5TimeoutMs(void)   { return g_q5_timeout_ms; }
uint32_t Route_GetQ5LapTimeMs(void)   { return g_q5_lap_time_ms; }
bool     Route_GetQ5PassedA(void)     { return g_q5_passed_a; }
