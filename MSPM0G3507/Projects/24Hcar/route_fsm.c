#include "route_fsm.h"

#include "buzzer.h"
#include "gray_sensor.h"
#include "motor.h"       /* Motor_* + Encoder_* */

/* ── 距离常量 ── */
#define ROUTE_AB_DISTANCE_CM      100.0f
#define ROUTE_BC_ARC_CM           110.0f
#define ROUTE_CD_DISTANCE_CM      100.0f
#define ROUTE_DA_ARC_CM           110.0f

/* ── 默认参数 ── */
#define ROUTE_ARC_PWM_DEFAULT     650
#define ROUTE_LINE_KP_DEFAULT     2.0f
#define ROUTE_LINE_KD_DEFAULT     0.0f
#define ROUTE_LINE_CENTER         350
#define ROUTE_HEADING_LIM         260
#define ROUTE_ARC_CORR_LIM        800
#define ROUTE_POINT_BEEP_MS       300U
#define ROUTE_POINT_WAIT_TICKS    60U

/* ── 状态枚举 ── */
typedef enum {
    ROUTE_STATE_IDLE = 0,
    /* MODE 5 弧线测试 */
    ROUTE_STATE_ARC_TEST,
    /* MODE 1 */
    ROUTE_STATE_AB_STRAIGHT,
    ROUTE_STATE_B_PROMPT,
    /* MODE 2 新增 */
    ROUTE_STATE_BC_ARC,
    ROUTE_STATE_C_PROMPT,
    ROUTE_STATE_CD_STRAIGHT,
    ROUTE_STATE_D_PROMPT,
    ROUTE_STATE_DA_ARC,
    ROUTE_STATE_A_PROMPT,
    /* 通用 */
    ROUTE_STATE_FINISHED,
    ROUTE_STATE_STOPPED
} RouteState_t;

/* ── 全局状态 ── */
static RouteMode_t  g_mode;
static RouteState_t g_state;
static uint16_t     g_wait_ticks;

/* 直线段参数 */
static float    g_target_yaw;
static bool     g_need_heading_lock;
static bool     g_yaw_target_set;     /* true = 用显式目标, false = 锁当前 */
static int16_t  g_straight_pwm;
static float    g_heading_kp;
static int16_t  g_straight_trim;
static int8_t   g_heading_sign;
static float    g_ab_yaw;             /* A→B 锁定的初始航向 */

/* 圆弧段参数 */
static int16_t  g_arc_pwm;
static float    g_line_kp;
static float    g_line_kd;
static float    g_arc_target_cm;

/* 圆弧内部状态 */
static float    g_arc_start_dist;
static float    g_line_prev_error;
static int16_t  g_arc_heading_lim;

/* ── 工具函数 ── */

static float Route_YawDiff(float current, float target)
{
    float diff = current - target;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

static int16_t Route_ClampPwm(int32_t value)
{
    if (value > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (value < 0) return 0;
    return (int16_t)value;
}

static int16_t Route_ClampCorrection(float value, int16_t limit)
{
    if (value > (float)limit)  return limit;
    if (value < (float)-limit) return -limit;
    return (int16_t)value;
}

/* ── 内部动作: 直线 ── */

static bool DriveStraightCm(float yaw_deg, float target_cm)
{
    if (g_need_heading_lock) {
        if (!g_yaw_target_set) {
            g_target_yaw = yaw_deg;    /* 无显式目标, 锁当前 */
        }
        g_need_heading_lock = false;
        g_yaw_target_set = false;
    }

    if (Encoder_GetAverageDistanceCm() >= target_cm) {
        Motor_Stop();
        return true;   /* 段完成 */
    }

    int16_t correction = Route_ClampCorrection(
        Route_YawDiff(yaw_deg, g_target_yaw) * g_heading_kp * (float)g_heading_sign,
        ROUTE_HEADING_LIM);
    int16_t left  = Route_ClampPwm((int32_t)g_straight_pwm + g_straight_trim - correction);
    int16_t right = Route_ClampPwm((int32_t)g_straight_pwm - g_straight_trim + correction);
    Motor_SetLeftSpeed(left);
    Motor_SetRightSpeed(right);
    return false;
}

/* ── 圆弧循迹内部状态 ── */
static float g_arc_accum_yaw;
static float g_arc_last_yaw;
static uint16_t g_arc_lost_cnt;    /* 全白持续计数 */

#define ARC_LOST_MAX_TICKS  60     /* 全白 300ms 后切 yaw 模式 */
#define ARC_YAW_KP          6.0f   /* yaw 补转比例 */

static bool FollowArcCm(float target_cm, float yaw_deg)
{
    float dist = Encoder_GetAverageDistanceCm() - g_arc_start_dist;

    /* 累计 yaw 变化 */
    float dyaw = yaw_deg - g_arc_last_yaw;
    g_arc_last_yaw = yaw_deg;
    if (dyaw > 180.0f)  dyaw -= 360.0f;
    if (dyaw < -180.0f) dyaw += 360.0f;
    g_arc_accum_yaw += dyaw;

    /* 退出条件: 距离达标 且 累计转角 >= 150° */
    bool dist_ok = (dist >= target_cm);
    bool yaw_ok  = (g_arc_accum_yaw >= 150.0f) || (g_arc_accum_yaw <= -150.0f);
    if (dist_ok && yaw_ok) {
        Motor_Stop();
        return true;
    }

    uint8_t raw = GraySensor_Read();
    int16_t pos = GraySensor_GetPosition(raw);

    /* ── 灰度全白检测 ── */
    if (pos < 0) {
        if (g_arc_lost_cnt < ARC_LOST_MAX_TICKS) {
            g_arc_lost_cnt++;
        }
    } else {
        g_arc_lost_cnt = 0;
    }

    /* ── 全白超时: 切 yaw 补转模式 ── */
    if (g_arc_lost_cnt >= ARC_LOST_MAX_TICKS) {
        /* 用 P 控制 yaw, 目标是累计转到 180° */
        float yaw_target;
        if (g_arc_accum_yaw > 0) {
            yaw_target = g_arc_last_yaw + (180.0f - g_arc_accum_yaw);
        } else {
            yaw_target = g_arc_last_yaw - (180.0f + g_arc_accum_yaw);
        }
        float yaw_err = Route_YawDiff(yaw_deg, yaw_target);
        int16_t corr = Route_ClampCorrection(yaw_err * ARC_YAW_KP, g_arc_heading_lim);
        int16_t left  = Route_ClampPwm((int32_t)g_arc_pwm + corr);
        int16_t right = Route_ClampPwm((int32_t)g_arc_pwm - corr);
        Motor_SetLeftSpeed(left);
        Motor_SetRightSpeed(right);
        return false;
    }

    /* ── 正常循迹模式 ── */
    float error;
    if (pos < 0) {
        /* 短暂丢线: 保持上一次误差方向 */
        error = (g_line_prev_error >= 0.0f) ? 250.0f : -250.0f;
    } else {
        error = (float)(pos - ROUTE_LINE_CENTER);   /* 正=偏右, 负=偏左 */
    }

    float diff_err = error - g_line_prev_error;
    g_line_prev_error = error;

    int16_t correction = Route_ClampCorrection(
        g_line_kp * error + g_line_kd * diff_err,
        g_arc_heading_lim);

    /* error>0 → 线偏右 → 左轮加速右转 → left+correction, right-correction */
    int16_t left  = Route_ClampPwm((int32_t)g_arc_pwm + correction);
    int16_t right = Route_ClampPwm((int32_t)g_arc_pwm - correction);
    Motor_SetLeftSpeed(left);
    Motor_SetRightSpeed(right);
    return false;
}

/* ── 参数设置 (公开) ── */

void Route_SetArcBase(int16_t pwm)
{
    if (pwm < 0) pwm = 0;
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    g_arc_pwm = pwm;
}

void Route_SetLineKp(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    g_line_kp = kp;
}

void Route_SetLineKd(float kd)
{
    g_line_kd = kd;
}

void Route_SetArcDistCm(float cm)
{
    if (cm < 10.0f) cm = 10.0f;
    g_arc_target_cm = cm;
}

/* ── 生命周期 ── */

void Route_Init(void)
{
    g_mode  = ROUTE_MODE_REQ1;
    g_state = ROUTE_STATE_STOPPED;

    g_target_yaw      = 0.0f;
    g_wait_ticks      = 0U;
    g_need_heading_lock = false;
    g_yaw_target_set  = false;
    g_ab_yaw          = 0.0f;
    g_straight_pwm    = 800;
    g_heading_kp      = 6.0f;
    g_straight_trim   = 30;
    g_heading_sign    = -1;

    g_arc_pwm         = ROUTE_ARC_PWM_DEFAULT;
    g_line_kp         = ROUTE_LINE_KP_DEFAULT;
    g_line_kd         = ROUTE_LINE_KD_DEFAULT;
    g_arc_target_cm   = ROUTE_BC_ARC_CM;
    g_arc_heading_lim = ROUTE_ARC_CORR_LIM;
    g_line_prev_error = 0.0f;
    g_arc_start_dist  = 0.0f;
}

void Route_SetMode(RouteMode_t mode)
{
    if ((mode < ROUTE_MODE_REQ1) || (mode > ROUTE_MODE_ARC_TEST)) return;
    g_mode = mode;
}

void Route_SetStraightBase(int16_t pwm)
{
    if (pwm < 0) pwm = 0;
    else if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    g_straight_pwm = pwm;
}

void Route_SetHeadingKp(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    g_heading_kp = kp;
}

void Route_SetStraightTrim(int16_t trim)
{
    g_straight_trim = trim;
}

void Route_SetHeadingSign(int8_t sign)
{
    g_heading_sign = (sign < 0) ? -1 : 1;
}

void Route_Start(void)
{
    Encoder_ResetDistance();
    g_wait_ticks      = 0U;
    g_line_prev_error = 0.0f;
    g_arc_start_dist  = 0.0f;

    if (g_mode == ROUTE_MODE_REQ1) {
        g_state = ROUTE_STATE_AB_STRAIGHT;
        g_need_heading_lock = true;
        Buzzer_Beep(ROUTE_POINT_BEEP_MS);
    } else if (g_mode == ROUTE_MODE_REQ2) {
        g_state = ROUTE_STATE_AB_STRAIGHT;
        g_need_heading_lock = true;
        Buzzer_Beep(ROUTE_POINT_BEEP_MS);
    } else if (g_mode == ROUTE_MODE_ARC_TEST) {
        g_state = ROUTE_STATE_ARC_TEST;
        g_arc_start_dist = Encoder_GetAverageDistanceCm();
        g_line_prev_error = 0.0f;
        g_arc_accum_yaw = 0.0f;
        g_arc_last_yaw = 0.0f;
        g_arc_lost_cnt = 0;
        Buzzer_Beep(ROUTE_POINT_BEEP_MS);
    } else {
        g_state = ROUTE_STATE_STOPPED;
        Motor_Stop();
        Buzzer_Beep(ROUTE_POINT_BEEP_MS);
    }
}

void Route_Stop(void)
{
    Motor_Stop();
    g_state = ROUTE_STATE_STOPPED;
    Buzzer_Beep(ROUTE_POINT_BEEP_MS);
}

/* ── 5ms 主循环 ── */

bool Route_Update5ms(float yaw_deg)
{
    Buzzer_Update5ms();

    switch (g_state) {

    /* ── 通用 ── */
    case ROUTE_STATE_IDLE:
        return false;
    case ROUTE_STATE_FINISHED:
    case ROUTE_STATE_STOPPED:
        Motor_Stop();
        return true;

    /* ── MODE 5: 单独弧线循迹测试 ── */
    case ROUTE_STATE_ARC_TEST:
        if (FollowArcCm(g_arc_target_cm, yaw_deg)) {
            Motor_Stop();
            Buzzer_Beep(ROUTE_POINT_BEEP_MS);
            g_state = ROUTE_STATE_FINISHED;
        }
        return true;

    /* ── MODE 1 & 2: A→B 直线 ── */
    case ROUTE_STATE_AB_STRAIGHT:
        g_ab_yaw = g_target_yaw;   /* 保存 A→B 航向 */
        if (DriveStraightCm(yaw_deg, ROUTE_AB_DISTANCE_CM)) {
            Buzzer_Beep(ROUTE_POINT_BEEP_MS);
            g_wait_ticks = 0U;
            if (g_mode == ROUTE_MODE_REQ1) {
                g_state = ROUTE_STATE_B_PROMPT;
            } else {
                /* MODE 2: 转入圆弧 B→C */
                g_arc_start_dist = Encoder_GetAverageDistanceCm();
                g_line_prev_error = 0.0f;
                g_arc_accum_yaw = 0.0f;
                g_arc_last_yaw = yaw_deg;
                g_arc_lost_cnt = 0;
                g_state = ROUTE_STATE_BC_ARC;
            }
        }
        return true;

    /* ── MODE 1: B 提示 → 结束 ── */
    case ROUTE_STATE_B_PROMPT:
        if (++g_wait_ticks >= ROUTE_POINT_WAIT_TICKS) {
            Motor_Stop();
            g_state = ROUTE_STATE_FINISHED;
        }
        return true;

    /* ── MODE 2: B→C 右半圆弧 ── */
    case ROUTE_STATE_BC_ARC:
        if (FollowArcCm(g_arc_target_cm, yaw_deg)) {
            Buzzer_Beep(ROUTE_POINT_BEEP_MS);
            g_wait_ticks = 0U;
            g_state = ROUTE_STATE_C_PROMPT;
        }
        return true;

    case ROUTE_STATE_C_PROMPT:
        if (++g_wait_ticks >= ROUTE_POINT_WAIT_TICKS) {
            Buzzer_Beep(ROUTE_POINT_BEEP_MS);
            Encoder_ResetDistance();
            /* C→D 目标 = A→B 航向 + 180° */
            g_target_yaw = g_ab_yaw + 180.0f;
            if (g_target_yaw > 180.0f) g_target_yaw -= 360.0f;
            g_need_heading_lock = true;
            g_yaw_target_set = true;
            g_state = ROUTE_STATE_CD_STRAIGHT;
        }
        return true;

    /* ── MODE 2: C→D 直线 ── */
    case ROUTE_STATE_CD_STRAIGHT:
        if (DriveStraightCm(yaw_deg, ROUTE_CD_DISTANCE_CM)) {
            Buzzer_Beep(ROUTE_POINT_BEEP_MS);
            g_wait_ticks = 0U;
            g_state = ROUTE_STATE_D_PROMPT;
        }
        return true;

    case ROUTE_STATE_D_PROMPT:
        if (++g_wait_ticks >= ROUTE_POINT_WAIT_TICKS) {
            Buzzer_Beep(ROUTE_POINT_BEEP_MS);
            /* 转入圆弧 D→A */
            g_arc_start_dist = Encoder_GetAverageDistanceCm();
            g_line_prev_error = 0.0f;
            g_arc_accum_yaw = 0.0f;
            g_arc_last_yaw = yaw_deg;
            g_arc_lost_cnt = 0;
            g_state = ROUTE_STATE_DA_ARC;
        }
        return true;

    /* ── MODE 2: D→A 左半圆弧 ── */
    case ROUTE_STATE_DA_ARC:
        if (FollowArcCm(g_arc_target_cm, yaw_deg)) {
            Buzzer_Beep(ROUTE_POINT_BEEP_MS);
            g_wait_ticks = 0U;
            g_state = ROUTE_STATE_A_PROMPT;
        }
        return true;

    case ROUTE_STATE_A_PROMPT:
        if (++g_wait_ticks >= ROUTE_POINT_WAIT_TICKS) {
            Motor_Stop();
            g_state = ROUTE_STATE_FINISHED;
        }
        return true;

    default:
        Motor_Stop();
        g_state = ROUTE_STATE_STOPPED;
        return true;
    }
}

/* ── 查询接口 ── */

bool Route_IsActive(void)
{
    switch (g_state) {
    case ROUTE_STATE_ARC_TEST:
    case ROUTE_STATE_AB_STRAIGHT:
    case ROUTE_STATE_B_PROMPT:
    case ROUTE_STATE_BC_ARC:
    case ROUTE_STATE_C_PROMPT:
    case ROUTE_STATE_CD_STRAIGHT:
    case ROUTE_STATE_D_PROMPT:
    case ROUTE_STATE_DA_ARC:
    case ROUTE_STATE_A_PROMPT:
        return true;
    default:
        return false;
    }
}

const char *Route_GetStateName(void)
{
    switch (g_state) {
    case ROUTE_STATE_IDLE:          return "IDLE";
    case ROUTE_STATE_ARC_TEST:      return "ARC ";
    case ROUTE_STATE_AB_STRAIGHT:   return "AB  ";
    case ROUTE_STATE_B_PROMPT:      return "B   ";
    case ROUTE_STATE_BC_ARC:        return "BC  ";
    case ROUTE_STATE_C_PROMPT:      return "C   ";
    case ROUTE_STATE_CD_STRAIGHT:   return "CD  ";
    case ROUTE_STATE_D_PROMPT:      return "D   ";
    case ROUTE_STATE_DA_ARC:        return "DA  ";
    case ROUTE_STATE_A_PROMPT:      return "A2  ";
    case ROUTE_STATE_FINISHED:      return "DONE";
    case ROUTE_STATE_STOPPED:       return "STOP";
    default:                        return "????";
    }
}
