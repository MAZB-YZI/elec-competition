/**
 * main.c 鈥?PID 宸＄嚎 + L 褰㈢洿瑙?+ 钃濈墮璋冨弬
 *
 * 鐘舵€? NORMAL / TURN_L / TURN_R
 * 钃濈墮: HC-05 鈫?UART3 (PB2/TX, PB3/RX)
 */

#include "ti_msp_dl_config.h"
#include "gray_sensor.h"
#include "motor.h"
#include "oled.h"
#include "delay.h"
#include "bluetooth.h"
#include "jy61p.h"
#include "ir_sensor.h"
#include "buzzer.h"
#include "route_fsm.h"
#include "mpu6050.h"
#include "soft_i2c.h"

/* ========== 榛樿鍙傛暟 (钃濈墮鍙敼) ========== */
#define DEAD_ZONE   3             /* 浣嶇疆姝诲尯: 卤3 鍐呬笉璋?*/
#define LOST_MS     500           /* 涓㈢嚎瓒呮椂 ms */
#define TURN_TARGET 90.0f        /* 鐩磋鐩爣瑙掑害 (搴? */
#define TURN_TIMEOUT 2000         /* 鐩磋瓒呮椂淇濇姢 ms */
#define TURN_SPEED_H 1500         /* 鐩磋杞集 PWM */
#define STEER_SLEW_STEP 75        /* 5ms 鍐呮渶澶ц浆鍚戝彉鍖栵紝鎻愰珮鍝嶅簲閫熷害 */
#define TURN_COOLDOWN_TICKS 40    /* 鐩磋閫€鍑哄悗鍐峰嵈 200ms锛岄槻姝簩娆¤Е鍙?*/
#define TURN_FORWARD_PULSES 400   /* 杞集鍓嶅墠杩涚紪鐮佸櫒鑴夊啿鏁?*/
#define CONTROL_DT 0.005f         /* 鎺у埗鍛ㄦ湡 5ms */
#define LEFT_ENCODER_DIR   1      /* 宸﹁疆缂栫爜鍣ㄦ柟鍚戠郴鏁?*/
#define RIGHT_ENCODER_DIR -1     /* 鍙宠疆缂栫爜鍣ㄦ柟鍚戠郴鏁帮紝鍙宠疆瀹夎鍙嶅悜 */
#define SPEED_LOOP_ENABLE 0       /* 0=寮€鐜祴璇曠紪鐮佸櫒锛?=鍚敤閫熷害PID */
#define SPEED_TARGET_SCALE 1.0f   /* PWM鎸囦护鍒扮紪鐮佸櫒閫熷害璁惧畾鍊肩殑姣斾緥 */
#define TURN_MIN_SPEED 420        /* 瑙掑害鐜浆寮椂鏈€浣嶱WM */
#define MPU6050_ADDR             0x68U
#define MPU6050_WHO_AM_I_REG     0x75U
#define LINE_STEER_SIGN          (1)
#define LINE_CENTER              350
#define LINE_SEARCH_STEER        420

static volatile float   g_KP         = 1.8f;   /* 浣嶇疆姣斾緥锛屼腑绛夊搷搴?*/
static volatile float   g_KI         = 0.0f;   /* 浣嶇疆绉垎锛屽厛鍏虫帀 */
static volatile float   g_KD         = 0.0f;   /* 浣嶇疆寰垎锛岄粯璁ゅ叧闂?*/
static volatile int16_t g_BASE_PWM   = 800;    /* 鐩磋鍩哄噯 PWM */
static volatile int16_t g_TURN_SPEED = 650;    /* 钃濈墮鍙皟杞集閫熷害 */
static volatile int16_t g_OUTPUT_LIM = 1000;   /* 浣嶇疆 PID 杈撳嚭闄愬箙 */

/* 鑸悜涓幆 PID 鍙傛暟 */
#define HEADING_KP      2.5f
#define HEADING_KI      0.0f
#define HEADING_KD      0.0f
#define HEADING_LIM     800     /* 鑸悜 PID 杈撳嚭闄愬箙 */

/* 杞﹁疆閫熷害鐜弬鏁帮紝璁惧畾鍊肩敱鏈€缁圥WM鎸囦护鎺ㄥ */
#define SPEED_KP        0.25f
#define SPEED_KI        0.05f
#define SPEED_KD        0.0f
#define SPEED_LIM       900

/* 90掳鐩磋杞集瑙掑害鐜弬鏁?*/
#define TURN_ANGLE_KP   10.0f
#define TURN_ANGLE_KI   0.0f
#define TURN_ANGLE_KD   0.6f
#define TURN_ANGLE_LIM  900

/* 鐘舵€佹満 */
typedef enum { NORMAL, TURN_WAIT, TURN_L, TURN_R } State_t;
static int16_t g_pos_filt;              /* 婊ゆ尝鍚庣殑浣嶇疆璇樊 */

/* ISR 鈫?main 鍏变韩 */
static volatile uint8_t  g_raw;          /* 鐏板害鍘熷 8-bit */
static volatile int16_t  g_pos;          /* 榛戠嚎璐ㄥ績 0~700 */
static volatile int16_t  g_steer;        /* 褰撳墠杞悜淇閲?*/
static volatile float    g_yaw;          /* 褰撳墠鑸悜瑙?*/
static volatile State_t  g_state;        /* 鐘舵€佹満: NORMAL/TURN_L/TURN_R */
static volatile bool     g_new_data;     /* ISR 鏂版暟鎹爣蹇?*/
static volatile uint32_t g_ms_ticks;
static volatile uint32_t g_systick_ms;
static volatile bool     g_mpu_ready;
static volatile bool     g_mpu_read_ok;

static PID_t      g_pid;                 /* 浣嶇疆 PID (澶栫幆) */
static PID_t      g_heading_pid;         /* 鑸悜 PID (涓幆) */
static PID_t      g_speed_pid_l;         /* 宸﹁疆閫熷害鐜?*/
static PID_t      g_speed_pid_r;         /* 鍙宠疆閫熷害鐜?*/
static PID_t      g_turn_angle_pid;      /* 鐩磋杞集瑙掑害鐜?*/
static float      g_target_yaw;          /* 鐩爣鑸悜瑙?(搴? */
static int16_t    g_last_steer;          /* 涓婃杞悜閲?涓㈢嚎淇濇寔鐢? */
static int16_t    g_last_pos_ctrl;       /* 涓婃浣嶇疆璇樊锛岀敤浜?D 椤?*/
static uint32_t   g_lost_cnt;            /* 涓㈢嚎鎸佺画璁℃暟 */
static uint32_t   g_turn_ticks;          /* 杞集鎸佺画 5ms 璁℃暟 */
static uint32_t   g_turn_confirm_ticks;  /* 鐩磋閫€鍑虹‘璁よ鏁?*/
static int32_t    g_last_enc_l, g_last_enc_r; /* 缂栫爜鍣ㄤ笂娆″€?*/
static volatile int16_t g_speed_l, g_speed_r; /* 缂栫爜鍣ㄩ€熷害 鑴夊啿/绉?*/
static uint32_t   g_turn_cooldown;            /* 鐩磋閫€鍑哄喎鍗磋鏁?*/
static float      g_turn_start_yaw;           /* 鐩磋璧峰鑸悜 */
static State_t    g_pending_turn;             /* 绛夊緟涓殑杞集鏂瑰悜 */
static uint32_t   g_all_white_cnt;            /* 鍏ㄧ櫧鎸佺画璁℃暟 */
static int32_t    g_turn_start_l, g_turn_start_r; /* 杞集绛夊緟缂栫爜鍣ㄨ捣濮嬪€?*/
static float      g_accumulated_angle;        /* 褰撳墠杞集绱瑙掑害 */
static float      g_last_yaw_for_accum;       /* 涓婃yaw锛岀敤浜庤绠楀閲?*/
static float      g_total_angle;              /* 鍏ㄥ眬绱瑙掑害锛屾寔缁疮鍔犱笉娓呴浂 */
static float      g_last_yaw_for_total;       /* 涓婃yaw锛岀敤浜庡叏灞€绱 */
static uint8_t    g_mpu_who;

void SysTick_Handler(void)
{
    ++g_systick_ms;
}

/* ================================================================
 *  杈呭姪
 * ================================================================ */
static inline uint8_t black(uint8_t raw, uint8_t i) { return (raw >> i) & 1; }

static State_t detect_turn(uint8_t raw)
{
    /* 鏀惧鏉′欢: 鏈€宸︾湅鍒伴粦绾匡紝鍙宠竟2涓负鐧藉嵆鍙?*/
    if (black(raw,0) && !black(raw,6) && !black(raw,7))
        return TURN_L;
    /* 鏀惧鏉′欢: 鏈€鍙崇湅鍒伴粦绾匡紝宸﹁竟2涓负鐧藉嵆鍙?*/
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

/* 璁＄畻鑸悜宸€硷紝澶勭悊 卤180掳 璺冲彉 */
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

static bool MPU6050_ReadWhoRetry(uint8_t *who)
{
    if (who == 0) {
        return false;
    }

    for (uint8_t attempt = 0U; attempt < 10U; ++attempt) {
        SoftI2C_Init();
        delay_ms(20U);
        if (SoftI2C_ProbeAddress(MPU6050_ADDR) &&
            SoftI2C_ReadReg(MPU6050_ADDR, MPU6050_WHO_AM_I_REG, who)) {
            return true;
        }
        delay_ms(30U);
    }

    return false;
}

static bool MPU6050_Bringup(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 CAR", 16);
    OLED_ShowString(0, 20, "READ WHO...", 12);
    OLED_Refresh();

    if (!MPU6050_ReadWhoRetry(&g_mpu_who)) {
        OLED_Clear();
        OLED_ShowString(0, 0, "WHO FAIL", 12);
        OLED_ShowString(0, 16, "ADDR 0x68", 12);
        OLED_ShowString(0, 32, "CHECK PA0 PA1", 12);
        OLED_Refresh();
        return false;
    }

    if (!MPU6050_Init()) {
        OLED_Clear();
        OLED_ShowString(0, 0, "MPU INIT FAIL", 12);
        OLED_ShowString(0, 16, "WHO:", 12);
        OLED_ShowNum(36, 16, g_mpu_who, 3, 12);
        OLED_Refresh();
        return false;
    }

    g_mpu_who = MPU6050_GetWhoAmI();
    OLED_Clear();
    OLED_ShowString(0, 0, "CALIBRATING", 12);
    OLED_ShowString(0, 16, "KEEP STILL", 12);
    OLED_ShowString(0, 32, "WHO:", 12);
    OLED_ShowNum(36, 32, g_mpu_who, 3, 12);
    OLED_Refresh();

    delay_ms(500U);
    if (!MPU6050_CalibrateGyro(1000U)) {
        OLED_Clear();
        OLED_ShowString(0, 0, "CAL FAIL", 12);
        OLED_ShowString(0, 16, "ERR:", 12);
        OLED_ShowNum(36, 16, MPU6050_GetReadErrorCount(), 5, 12);
        OLED_Refresh();
        return false;
    }

    MPU6050_ResetYaw();
    g_mpu_ready = true;
    g_mpu_read_ok = true;
    return true;
}

static void MPU6050_Service(void)
{
    static uint32_t last_update_ms;
    uint32_t now = g_systick_ms;
    uint32_t elapsed_ms = now - last_update_ms;

    if (!g_mpu_ready) {
        last_update_ms = now;
        return;
    }

    if (elapsed_ms == 0U) {
        return;
    }

    last_update_ms = now;
    g_mpu_read_ok = MPU6050_Update((float) elapsed_ms / 1000.0f);
}
/* ================================================================
 *  TIMG6 ISR: 5ms 鎺у埗
 * ================================================================ */
void CTRL_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(CTRL_TIMER_INST)) {
    case DL_TIMER_IIDX_LOAD: {
        g_ms_ticks += 5U;
        Buzzer_Update5ms();

        /* 缂栫爜鍣ㄦ祴閫燂紙鏂瑰悜褰掍竴鍖栵細鍓嶈繘鏃堕兘涓烘锛?*/
        int32_t enc_l = Encoder_GetLeftCount();
        int32_t enc_r = Encoder_GetRightCount();
        g_speed_l = (int16_t)((enc_l - g_last_enc_l) * 200 * LEFT_ENCODER_DIR);
        g_speed_r = (int16_t)((enc_r - g_last_enc_r) * 200 * RIGHT_ENCODER_DIR);
        g_last_enc_l = enc_l; g_last_enc_r = enc_r;

        float   yaw   = MPU6050_GetYaw();

        /* 鍏ㄥ眬绱瑙掑害锛氭瘡5ms绱姞yaw鍙樺寲锛岀敤浜庤绠楁€诲湀鏁?*/
        float total_delta = yaw_diff(yaw, g_last_yaw_for_total);
        g_last_yaw_for_total = yaw;
        g_total_angle += total_delta;

        /* ---- Route 鐘舵€佹満浼樺厛 (寮х嚎寰抗鍦?route_fsm.c 鍐呰鐏板害) ---- */
        if (Route_IsActive() && Route_Update5ms(yaw)) {
            g_steer = 0; g_yaw = yaw;
            g_new_data = true;
            break;
        }

        /* ---- 闈?Route 鐘舵€? 璇荤伆搴﹀仛鏅€氬贰绾?---- */
        uint8_t raw  = GraySensor_Read();

        int16_t pos = GraySensor_GetPosition(raw);

        int16_t steer = 0;
        int16_t base  = g_BASE_PWM;
        int16_t turn  = g_TURN_SPEED;
        int16_t lim   = g_OUTPUT_LIM;

        State_t st = g_state;
        if (st == NORMAL) {
            /* 鍐峰嵈鏈熷唴涓嶆娴嬬洿瑙掞紝闃叉閫€鍑哄悗绔嬪嵆浜屾瑙﹀彂 */
            if (g_turn_cooldown > 0) {
                g_turn_cooldown--;
            } else {
                State_t next = detect_turn(raw);
                if (next != NORMAL) {
                    st = TURN_WAIT;
                    g_pending_turn = next;        /* 璁板綍杞集鏂瑰悜 */
                    g_turn_start_l = enc_l;
                    g_turn_start_r = enc_r;
                    g_turn_ticks = 0;
                    g_all_white_cnt = 0;
                    reset_motion_pids();
                }
            }
        } else if (st == TURN_WAIT) {
            /* 杞集鍓嶅墠杩涘浐瀹氱紪鐮佸櫒璺濈 */
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
            /* 瓒呮椂淇濇姢锛氱紪鐮佸櫒寮傚父鎴栬溅杈嗗崱浣忔椂 */
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
            /* 姣?ms绱姞yaw鍙樺寲閲忥紝澶勭悊卤180掳璺冲彉 */
            float turned = yaw_diff(yaw, g_last_yaw_for_accum);
            g_last_yaw_for_accum = yaw;
            if (turned < 0) turned = -turned;
            g_accumulated_angle += turned;

            /* 閫€鍑烘潯浠? 绱瑙掑害杈惧埌鐩爣 */
            bool angle_done = (g_accumulated_angle >= TURN_TARGET);
            /* 瓒呮椂淇濇姢: 闃叉鍗℃ */
            bool timeout = (g_turn_ticks > (TURN_TIMEOUT / 5));

            /* 浼犳劅鍣ㄩ€€鍑? 杞60掳鍚庢壂鍒板闈㈢嚎鎵嶅仠 */
            bool sensor_exit = false;
            if (g_accumulated_angle > 60.0f) {
                if (st == TURN_R && black(raw, 1))  /* 鍙宠浆: 宸﹁竟绗簩涓湅鍒伴粦绾?*/
                    sensor_exit = true;
                if (st == TURN_L && black(raw, 6))  /* 宸﹁浆: 鍙宠竟绗簩涓湅鍒伴粦绾?*/
                    sensor_exit = true;
            }

            if (angle_done || timeout || sensor_exit) {
                st = NORMAL;
                g_turn_cooldown = TURN_COOLDOWN_TICKS;  /* 鍚姩鍐峰嵈 200ms */
                /* 閫€鍑虹洿瑙? 閲嶇疆鐩爣鑸悜涓哄綋鍓嶅疄闄呰埅鍚? 闃茬獊鍙?*/
                g_target_yaw = yaw;
                PID_Reset(&g_heading_pid);
                reset_motion_pids();
            }
        }
        g_state = st;

        /* ---- 鎵ц ---- */
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
            /* 绛夊緟鍏ㄧ櫧: 淇濇寔鐩磋 */
            drive_closed_loop(base, base);
            steer = 0;
        } else {
            /* NORMAL: 鐏板害 P 宸＄嚎 */
            bool all_black = (raw == 0xFF);
            bool has_line = (pos >= 0) && !all_black;

            if (has_line && !all_black) {
                /* 鏈夌嚎涓旈潪鍏ㄩ粦: 姝ｅ父宸＄嚎 */
                int16_t pos_ctrl = (int16_t)(pos - LINE_CENTER);
                if (pos_ctrl > -DEAD_ZONE && pos_ctrl < DEAD_ZONE) {
                    pos_ctrl = 0;
                }

                int16_t d_pos = pos_ctrl - g_last_pos_ctrl;
                steer = (int16_t)((float)LINE_STEER_SIGN *
                                  ((float)pos_ctrl * g_KP + (float)d_pos * g_KD));
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
                /* 鍏ㄧ櫧鎴栧叏榛? 淇濇寔涓婃杞悜, 瓒呮椂鍋滆溅 */
                g_last_pos_ctrl = 0;
                steer = (g_last_steer != 0) ? g_last_steer : LINE_SEARCH_STEER;
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

/* ================================================================
 *  main
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
    Motor_Init();
    Buzzer_Init();
    Route_Init();
    OLED_Init();
    if (!MPU6050_Bringup()) {
        Motor_Stop();
        Buzzer_Stop();
        while (1) {
        }
    }
    PID_Init(&g_pid, g_KP, g_KI, g_KD, g_OUTPUT_LIM);
    PID_Init(&g_heading_pid, HEADING_KP, HEADING_KI, HEADING_KD, HEADING_LIM);
    PID_Init(&g_speed_pid_l, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_LIM);
    PID_Init(&g_speed_pid_r, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_LIM);
    PID_Init(&g_turn_angle_pid, TURN_ANGLE_KP, TURN_ANGLE_KI, TURN_ANGLE_KD, TURN_ANGLE_LIM);
    g_target_yaw = 0.0f;
    g_total_angle = 0.0f;
    g_last_yaw_for_total = MPU6050_GetYaw();  /* 璁板綍鍒濆鑸悜 */
    BT_Init();
    BT_SetTickPtr(&g_ms_ticks);
    BT_Send("MPU6050 Car Ready\r\n");

    OLED_Clear();
    OLED_ShowString(0, 0, "LinerCar BT", 16);
    OLED_Clear();
    OLED_Refresh();

    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);

    TuningParams_t bt_params = { g_KP, g_KI, g_BASE_PWM, g_TURN_SPEED, g_OUTPUT_LIM };
    uint32_t tick = 0;

    while (1) {
        MPU6050_Service();

        /* 钃濈墮璋冨弬 */
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
                /* 鏄剧ず鍏ㄥ眬绱鍦堟暟锛堥檧铻轰华鎸佺画绱姞锛?*/
                OLED_ShowString(36, 18, "P", 12);
                OLED_ShowSigned4(48, 18, pos);

                int32_t speed_diff = (int32_t)g_speed_l - (int32_t)g_speed_r;
                OLED_ShowString(0, 38, "L", 12);
                OLED_ShowSigned4(8, 38, g_speed_l);
                OLED_ShowString(56, 38, "R", 12);
                OLED_ShowSigned4(64, 38, g_speed_r);
                
                /* 搴曡: 濮嬬粓鏄剧ず閫熷害宸拰鑸悜 */
                float yaw = g_yaw;
                OLED_ShowString(0, 52, "D", 12);
                OLED_ShowSigned4(8, 52, speed_diff);
                OLED_ShowString(64, 52, "Y", 12);
                int32_t yaw_int = (int32_t)(yaw * 10);
                OLED_ShowSigned4(72, 52, yaw_int / 10);
                OLED_ShowString(108, 52, g_mpu_read_ok ? "O" : "E", 12);

                OLED_Refresh();
            }
        }
    }
}
