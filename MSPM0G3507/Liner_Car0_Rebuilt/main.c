/**
 * main.c — PID 巡线 + L 形直角 + 蓝牙调参
 *
 * 状态: NORMAL / TURN_L / TURN_R
 * 蓝牙: HC-05 → UART3 (PB2/TX, PB3/RX)
 */

#include "ti_msp_dl_config.h"
#include "gray_sensor.h"
#include "motor.h"
#include "oled.h"
#include "delay.h"
#include "bluetooth.h"
#include "jy61p.h"
#include "ir_sensor.h"

/* ========== 默认参数 (蓝牙可改) ========== */
#define DEAD_ZONE   3             /* 位置死区: ±3 内不调 */
#define LOST_MS     500           /* 丢线超时 ms */
#define TURN_TARGET 90.0f        /* 直角目标角度 (度) */
#define TURN_TIMEOUT 1500         /* 直角超时保护 ms */
#define TURN_SPEED_H 1500         /* 直角转弯 PWM */
#define STEER_SLEW_STEP 75        /* 5ms 内最大转向变化，提高响应速度 */
#define TURN_COOLDOWN_TICKS 40    /* 直角退出后冷却 200ms，防止二次触发 */
#define TURN_FORWARD_PULSES 400   /* 转弯前前进编码器脉冲数 */
#define CONTROL_DT 0.005f         /* 控制周期 5ms */
#define LEFT_ENCODER_DIR   1      /* 左轮编码器方向系数 */
#define RIGHT_ENCODER_DIR -1     /* 右轮编码器方向系数，右轮安装反向 */
#define SPEED_LOOP_ENABLE 0       /* 0=开环测试编码器，1=启用速度PID */
#define SPEED_TARGET_SCALE 1.0f   /* PWM指令到编码器速度设定值的比例 */
#define TURN_MIN_SPEED 420        /* 角度环转弯时最低PWM */

static volatile float   g_KP         = 1.8f;   /* 位置比例，中等响应 */
static volatile float   g_KI         = 0.0f;   /* 位置积分，先关掉 */
static volatile float   g_KD         = 0.0f;   /* 位置微分，默认关闭 */
static volatile int16_t g_BASE_PWM   = 600;    /* 直行基准 PWM，降速防甩出 */
static volatile int16_t g_TURN_SPEED = 650;    /* 蓝牙可调转弯速度 */
static volatile int16_t g_OUTPUT_LIM = 1000;   /* 位置 PID 输出限幅 */

/* 航向中环 PID 参数 */
#define HEADING_KP      2.5f
#define HEADING_KI      0.0f
#define HEADING_KD      0.0f
#define HEADING_LIM     800     /* 航向 PID 输出限幅 */

/* 车轮速度环参数，设定值由最终PWM指令推导 */
#define SPEED_KP        0.25f
#define SPEED_KI        0.05f
#define SPEED_KD        0.0f
#define SPEED_LIM       900

/* 90°直角转弯角度环参数 */
#define TURN_ANGLE_KP   10.0f
#define TURN_ANGLE_KI   0.0f
#define TURN_ANGLE_KD   0.6f
#define TURN_ANGLE_LIM  900

/* 状态机 */
typedef enum { NORMAL, TURN_WAIT, TURN_L, TURN_R } State_t;
static int16_t g_pos_filt;              /* 滤波后的位置误差 */

/* ISR ↔ main 共享 */
static volatile uint8_t  g_raw;          /* 灰度原始 8-bit */
static volatile int16_t  g_pos;          /* 黑线质心 0~700 */
static volatile int16_t  g_steer;        /* 当前转向修正量 */
static volatile float    g_yaw;          /* 当前航向角 */
static volatile State_t  g_state;        /* 状态机: NORMAL/TURN_L/TURN_R */
static volatile bool     g_new_data;     /* ISR 新数据标志 */

static PID_t      g_pid;                 /* 位置 PID (外环) */
static PID_t      g_heading_pid;         /* 航向 PID (中环) */
static PID_t      g_speed_pid_l;         /* 左轮速度环 */
static PID_t      g_speed_pid_r;         /* 右轮速度环 */
static PID_t      g_turn_angle_pid;      /* 直角转弯角度环 */
static float      g_target_yaw;          /* 目标航向角 (度) */
static int16_t    g_last_steer;          /* 上次转向量(丢线保持用) */
static int16_t    g_last_pos_ctrl;       /* 上次位置误差，用于 D 项 */
static uint32_t   g_lost_cnt;            /* 丢线持续计数 */
static uint32_t   g_turn_ticks;          /* 转弯持续 5ms 计数 */
static uint32_t   g_turn_confirm_ticks;  /* 直角退出确认计数 */
static int32_t    g_last_enc_l, g_last_enc_r; /* 编码器上次值 */
static volatile int16_t g_speed_l, g_speed_r; /* 编码器速度 脉冲/秒 */
static uint32_t   g_turn_cooldown;            /* 直角退出冷却计数 */
static float      g_turn_start_yaw;           /* 直角起始航向 */
static State_t    g_pending_turn;             /* 等待中的转弯方向 */
static uint32_t   g_all_white_cnt;            /* 全白持续计数 */
static int32_t    g_turn_start_l, g_turn_start_r; /* 转弯等待编码器起始值 */
static float      g_accumulated_angle;        /* 当前转弯累计角度 */
static float      g_last_yaw_for_accum;       /* 上次yaw，用于计算增量 */
static float      g_total_angle;              /* 全局累计角度，持续累加不清零 */
static float      g_last_yaw_for_total;       /* 上次yaw，用于全局累计 */

/* ================================================================
 *  辅助
 * ================================================================ */
static inline uint8_t black(uint8_t raw, uint8_t i) { return (raw >> i) & 1; }

static State_t detect_turn(uint8_t raw)
{
    /* 放宽条件: 最左看到黑线，右边2个为白即可 */
    if (black(raw,0) && !black(raw,6) && !black(raw,7))
        return TURN_L;
    /* 放宽条件: 最右看到黑线，左边2个为白即可 */
    if (black(raw,7) && !black(raw,0) && !black(raw,1))
        return TURN_R;
    return NORMAL;
}

static bool turn_done(uint8_t raw)
{
    uint8_t mid = black(raw,2) + black(raw,3)
                + black(raw,4) + black(raw,5);
    return (mid >= 2);
}

/* 计算航向差值，处理 ±180° 跳变 */
static float yaw_diff(float current, float start)
{
    float diff = current - start;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

static int16_t clamp_pwm(int32_t value)
{
    if (value > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (value < MOTOR_PWM_MIN) return MOTOR_PWM_MIN;
    return (int16_t)value;
}

static int16_t abs16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static int16_t apply_speed_loop(PID_t *pid, int16_t pwm_cmd, int16_t measured_speed)
{
    int16_t setpoint = (int16_t)((float)pwm_cmd * SPEED_TARGET_SCALE);
    int16_t correction = PID_Compute(pid, setpoint, measured_speed, CONTROL_DT);
    return clamp_pwm((int32_t)pwm_cmd + correction);
}

static void drive_closed_loop(int16_t left_pwm, int16_t right_pwm)
{
#if SPEED_LOOP_ENABLE
    int16_t left_out = apply_speed_loop(&g_speed_pid_l, left_pwm, g_speed_l);
    int16_t right_out = apply_speed_loop(&g_speed_pid_r, right_pwm, g_speed_r);
    Motor_SetLeftSpeed(left_out);
    Motor_SetRightSpeed(right_out);
#else
    Motor_SetLeftSpeed(clamp_pwm(left_pwm));
    Motor_SetRightSpeed(clamp_pwm(right_pwm));
#endif
}

static void reset_motion_pids(void)
{
    PID_Reset(&g_speed_pid_l);
    PID_Reset(&g_speed_pid_r);
    PID_Reset(&g_turn_angle_pid);
}

static void OLED_ShowSigned4(uint8_t x, uint8_t y, int32_t value)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        value = -value;
    } else {
        OLED_ShowString(x, y, "+", 12);
    }
    OLED_ShowNum((uint8_t)(x + 6), y, (uint32_t)value, 4, 12);
}
/* ================================================================
 *  TIMG6 ISR: 5ms 控制
 * ================================================================ */
void CTRL_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(CTRL_TIMER_INST)) {
    case DL_TIMER_IIDX_LOAD: {
        JY61P_UpdateTick();

        /* 编码器测速（方向归一化：前进时都为正） */
        int32_t enc_l = Encoder_GetLeftCount();
        int32_t enc_r = Encoder_GetRightCount();
        g_speed_l = (int16_t)((enc_l - g_last_enc_l) * 200 * LEFT_ENCODER_DIR);
        g_speed_r = (int16_t)((enc_r - g_last_enc_r) * 200 * RIGHT_ENCODER_DIR);
        g_last_enc_l = enc_l; g_last_enc_r = enc_r;

        uint8_t raw  = GraySensor_Read();

        /* 加权连续位置: 外层权重高 → 更细腻, 不会蹦 100 一跳 */
        int8_t s[8]; uint8_t i;
        for (i = 0; i < 8; i++) s[i] = (raw >> i) & 1;
        int16_t pos = ( 200)*s[0] + (140)*s[1] + ( 75)*s[2] + ( 40)*s[3]
                    + (-40)*s[4] + (-75)*s[5] + (-140)*s[6] + (-200)*s[7];

        int16_t steer = 0;
        int16_t base  = g_BASE_PWM;
        int16_t turn  = g_TURN_SPEED;
        int16_t lim   = g_OUTPUT_LIM;
        float   yaw   = JY61P_GetYaw();

        /* 全局累计角度：每5ms累加yaw变化，用于计算总圈数 */
        float total_delta = yaw_diff(yaw, g_last_yaw_for_total);
        g_last_yaw_for_total = yaw;
        g_total_angle += total_delta;

        /* ---- 状态机: 直角检测 ---- */
        State_t st = g_state;
        if (st == NORMAL) {
            /* 冷却期内不检测直角，防止退出后立即二次触发 */
            if (g_turn_cooldown > 0) {
                g_turn_cooldown--;
            } else {
                State_t next = detect_turn(raw);
                if (next != NORMAL) {
                    st = TURN_WAIT;
                    g_pending_turn = next;        /* 记录转弯方向 */
                    g_turn_start_l = enc_l;
                    g_turn_start_r = enc_r;
                    g_turn_ticks = 0;
                    g_all_white_cnt = 0;
                    reset_motion_pids();
                }
            }
        } else if (st == TURN_WAIT) {
            /* 转弯前前进固定编码器距离 */
            g_turn_ticks++;
            int32_t dl = enc_l - g_turn_start_l;
            int32_t dr = enc_r - g_turn_start_r;
            if (dl < 0) dl = -dl;
            if (dr < 0) dr = -dr;
            int32_t forward_pulses = (dl + dr) / 2;

            if (forward_pulses >= TURN_FORWARD_PULSES) {
                st = g_pending_turn;
                g_turn_ticks = 0;
                g_accumulated_angle = 0;
                g_turn_start_yaw = yaw;
                g_last_yaw_for_accum = yaw;
                g_all_white_cnt = 0;
                reset_motion_pids();
            }
            /* 超时保护：编码器异常或车辆卡住时 */
            if (g_turn_ticks > 300) {  /* 1500ms */
                st = g_pending_turn;
                g_turn_ticks = 0;
                g_accumulated_angle = 0;
                g_turn_start_yaw = yaw;
                g_last_yaw_for_accum = yaw;
                g_all_white_cnt = 0;
                reset_motion_pids();
            }
        } else {
            g_turn_ticks++;
            /* 每5ms累加yaw变化量，处理±180°跳变 */
            float turned = yaw_diff(yaw, g_last_yaw_for_accum);
            g_last_yaw_for_accum = yaw;
            if (turned < 0) turned = -turned;
            g_accumulated_angle += turned;

            /* 退出条件: 累计角度达到目标 */
            bool angle_done = (g_accumulated_angle >= TURN_TARGET);
            /* 超时保护: 防止卡死 */
            bool timeout = (g_turn_ticks > (TURN_TIMEOUT / 5));

            /* 传感器退出: 转够60°后扫到对面线才停 */
            bool sensor_exit = false;
            if (g_accumulated_angle > 60.0f) {
                if (st == TURN_R && black(raw, 1))  /* 右转: 左边第二个看到黑线 */
                    sensor_exit = true;
                if (st == TURN_L && black(raw, 6))  /* 左转: 右边第二个看到黑线 */
                    sensor_exit = true;
            }

            if (angle_done || timeout || sensor_exit) {
                st = NORMAL;
                g_turn_cooldown = TURN_COOLDOWN_TICKS;  /* 启动冷却 200ms */
                /* 退出直角: 重置目标航向为当前实际航向, 防突变 */
                g_target_yaw = yaw;
                PID_Reset(&g_heading_pid);
                reset_motion_pids();
            }
        }
        g_state = st;

        /* ---- 执行 ---- */
        if (st == TURN_L) {
            int16_t angle_speed = PID_Compute(&g_turn_angle_pid,
                                              (int16_t)TURN_TARGET,
                                              (int16_t)g_accumulated_angle,
                                              CONTROL_DT);
            angle_speed = abs16(angle_speed);
            if (angle_speed < TURN_MIN_SPEED) angle_speed = TURN_MIN_SPEED;
            if (angle_speed > turn) angle_speed = turn;
            drive_closed_loop(-angle_speed, angle_speed);
            steer = lim;
        } else if (st == TURN_R) {
            int16_t angle_speed = PID_Compute(&g_turn_angle_pid,
                                              (int16_t)TURN_TARGET,
                                              (int16_t)g_accumulated_angle,
                                              CONTROL_DT);
            angle_speed = abs16(angle_speed);
            if (angle_speed < TURN_MIN_SPEED) angle_speed = TURN_MIN_SPEED;
            if (angle_speed > turn) angle_speed = turn;
            drive_closed_loop(angle_speed, -angle_speed);
            steer = -lim;
        } else if (st == TURN_WAIT) {
            /* 等待全白: 保持直行 */
            drive_closed_loop(base, base);
            steer = 0;
        } else {
            /* NORMAL: 灰度 P 巡线 */
            bool all_white = (raw == 0);
            bool all_black = (raw == 0xFF);
            bool has_line = (s[0] || s[1] || s[2] || s[3] || s[4] || s[5] || s[6] || s[7]);

            if (has_line && !all_black) {
                /* 有线且非全黑: 正常巡线 */
                int16_t pos_ctrl = pos;
                if (pos_ctrl > -DEAD_ZONE && pos_ctrl < DEAD_ZONE) {
                    pos_ctrl = 0;
                }

                int16_t d_pos = pos_ctrl - g_last_pos_ctrl;
                steer = (int16_t)(-((float)pos_ctrl * g_KP + (float)d_pos * g_KD));
                g_last_pos_ctrl = pos_ctrl;
                g_pos_filt = pos_ctrl;
                if (steer >  lim) steer =  lim;
                if (steer < -lim) steer = -lim;

                int16_t delta = steer - g_last_steer;
                if (delta >  STEER_SLEW_STEP) steer = g_last_steer + STEER_SLEW_STEP;
                if (delta < -STEER_SLEW_STEP) steer = g_last_steer - STEER_SLEW_STEP;

                g_target_yaw = yaw;
                g_last_steer = steer;
                g_lost_cnt   = 0;
            } else {
                /* 全白或全黑: 保持上次转向, 超时停车 */
                g_last_pos_ctrl = 0;
                steer = g_last_steer;
                if (++g_lost_cnt > LOST_MS / 5) {
                    Motor_Stop(); steer = 0;
                    reset_motion_pids();
                }
            }
            drive_closed_loop(base + steer, base - steer);
        }

        g_raw = raw; g_pos = pos; g_steer = steer; g_yaw = yaw;
        g_new_data = true;
        break;
    }
    }
}

void GROUP1_IRQHandler(void) { Encoder_ISR(); }

/* UART0 中断 → JY61P 陀螺仪 */
void UART0_IRQHandler(void) { JY61P_UART_IRQHandler(); }

/* ================================================================
 *  main
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
    OLED_Init();
    PID_Init(&g_pid, g_KP, g_KI, g_KD, g_OUTPUT_LIM);
    PID_Init(&g_heading_pid, HEADING_KP, HEADING_KI, HEADING_KD, HEADING_LIM);
    PID_Init(&g_speed_pid_l, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_LIM);
    PID_Init(&g_speed_pid_r, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_LIM);
    PID_Init(&g_turn_angle_pid, TURN_ANGLE_KP, TURN_ANGLE_KI, TURN_ANGLE_KD, TURN_ANGLE_LIM);
    g_target_yaw = 0.0f;
    g_total_angle = 0.0f;
    g_last_yaw_for_total = JY61P_GetYaw();  /* 记录初始航向 */
    BT_Init();
    JY61P_Init();
    BT_Send("LinerCar Ready\r\n");

    OLED_Clear();
    OLED_ShowString(0, 0, "LinerCar BT", 16);
    OLED_Clear();
    OLED_Refresh();

    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);

    TuningParams_t bt_params = { g_KP, g_KI, g_BASE_PWM, g_TURN_SPEED, g_OUTPUT_LIM };
    uint32_t tick = 0;

    while (1) {
        /* 蓝牙调参 */
        if (BT_Poll(&bt_params)) {
            g_KP         = bt_params.KP;
            g_KI         = bt_params.KI;
            g_BASE_PWM   = bt_params.BASE_PWM;
            g_TURN_SPEED = bt_params.TURN_SPEED;
            g_OUTPUT_LIM = bt_params.OUTPUT_LIM;
            PID_Init(&g_pid, g_KP, g_KI, g_KD, g_OUTPUT_LIM);
        }

        /* OLED */
        if (g_new_data) {
            g_new_data = false;
            if (++tick % 100 == 0) {
                uint8_t  raw = g_raw;
                int16_t  pos = g_pos;
                State_t  st  = g_state;
                int16_t  spd_L, spd_R;
                if (st == TURN_L)      { spd_L =  g_TURN_SPEED; spd_R = -g_TURN_SPEED; }
                else if (st == TURN_R) { spd_L = -g_TURN_SPEED; spd_R =  g_TURN_SPEED; }
                else {
                    int16_t s = g_steer;
                    spd_L = g_BASE_PWM - s;
                    spd_R = g_BASE_PWM + s;
                }

                for (uint8_t i = 0; i < 8; i++)
                    OLED_ShowNum(i * 16, 0, (raw >> i) & 1, 1, 16);
                
                const char *ss = (st == NORMAL)    ? "NORM"
                               : (st == TURN_WAIT) ? "WAIT"
                               : (st == TURN_L)    ? "TL  " : "TR  ";
                OLED_ShowString(0, 18, ss, 12);
                /* 显示全局累计圈数（陀螺仪持续累加） */
                uint32_t total_turns_x10 = (uint32_t)((g_total_angle * 10.0f) / 360.0f);
                if (g_total_angle < 0) total_turns_x10 = (uint32_t)((-g_total_angle * 10.0f) / 360.0f);
                OLED_ShowString(36, 18, "R:", 12);
                OLED_ShowNum(48, 18, total_turns_x10 / 10, 1, 12);
                OLED_ShowString(56, 18, ".", 12);
                OLED_ShowNum(62, 18, total_turns_x10 % 10, 1, 12);

                int32_t speed_diff = (int32_t)g_speed_l - (int32_t)g_speed_r;
                OLED_ShowString(0, 38, "L", 12);
                OLED_ShowSigned4(8, 38, g_speed_l);
                OLED_ShowString(56, 38, "R", 12);
                OLED_ShowSigned4(64, 38, g_speed_r);
                
                /* 底行: 始终显示速度差和航向 */
                float yaw = g_yaw;
                OLED_ShowString(0, 52, "D", 12);
                OLED_ShowSigned4(8, 52, speed_diff);
                OLED_ShowString(64, 52, "Y", 12);
                int32_t yaw_int = (int32_t)(yaw * 10);
                OLED_ShowSigned4(72, 52, yaw_int / 10);

                OLED_Refresh();
            }
        }
    }
}
