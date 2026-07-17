/**
 * line_sensor.c — H_CAR 灰度循迹适配层
 *
 * 完全移植 Liner_Car0_Rebuilt 的循迹逻辑：
 * - 加权位置（200, 140, 75, 40, -40, -75, -140, -200）
 * - 死区 ±3
 * - PD 控制（KP=1.8, KD=0.0）
 * - 转向限速 75/周期
 * - 全白/全黑保持上次转向
 * - 丢线超时停车
 *
 * 返回值: steer（-lim ~ +lim），直接可加减到 base PWM
 */

#include "line_sensor.h"
#include "gray_sensor.h"

/* ========== 参数（与 Liner_Car0_Rebuilt 完全一致） ========== */
#define LINE_KP             1.8f
#define LINE_KD             0.0f
#define LINE_DEAD_ZONE      3
#define LINE_STEER_SLEW     75
#define LINE_LOST_TICKS     100     /* 500ms / 5ms */
#define LINE_OUTPUT_LIM     1000

/* ========== 内部状态 ========== */
static uint8_t last_raw = 0U;
static int16_t last_pos_ctrl = 0;
static int16_t last_steer = 0;
static uint32_t lost_count = 0U;
static bool has_line = false;

/* ========== LineSensor 接口实现 ========== */

/**
 * LineSensor_GetError — 返回转向量（steer）
 *
 * 完全复制 Liner_Car0_Rebuilt 的 NORMAL 巡线逻辑：
 * 1. 读传感器 raw
 * 2. 加权计算 pos
 * 3. 死区处理
 * 4. PD 控制
 * 5. clamp + slew rate
 *
 * 返回值: steer（-1000 ~ +1000），可直接加减到 base_pwm
 */
float LineSensor_GetError(void)
{
    uint8_t raw = GraySensor_Read();
    last_raw = raw;

    /* 加权位置（与 Liner_Car0_Rebuilt 完全一致） */
    int8_t s[8];
    for (uint8_t i = 0; i < 8; i++) {
        s[i] = (raw >> i) & 1;
    }
    int16_t pos = ( 200) * s[0]
                + ( 140) * s[1]
                + (  75) * s[2]
                + (  40) * s[3]
                + ( -40) * s[4]
                + ( -75) * s[5]
                + (-140) * s[6]
                + (-200) * s[7];

    /* 检测黑线 */
    bool any_black = false;
    bool all_white = true;
    for (uint8_t i = 0; i < 8; i++) {
        if ((raw >> i) & 1) {
            any_black = true;
            all_white = false;
        }
    }
    has_line = any_black && !all_white;

    if (!has_line) {
        lost_count++;
        /* 全白/全黑: 保持上次转向 */
        return (float)last_steer;
    }
    lost_count = 0;

    /* 死区 */
    int16_t pos_ctrl = pos;
    if (pos_ctrl > -LINE_DEAD_ZONE && pos_ctrl < LINE_DEAD_ZONE) {
        pos_ctrl = 0;
    }

    /* PD 控制 */
    int16_t d_pos = pos_ctrl - last_pos_ctrl;
    int16_t steer = (int16_t)(-((float)pos_ctrl * LINE_KP + (float)d_pos * LINE_KD));
    last_pos_ctrl = pos_ctrl;

    /* clamp */
    if (steer > LINE_OUTPUT_LIM) steer = LINE_OUTPUT_LIM;
    if (steer < -LINE_OUTPUT_LIM) steer = -LINE_OUTPUT_LIM;

    /* 转向限速 */
    int16_t delta = steer - last_steer;
    if (delta > LINE_STEER_SLEW) steer = last_steer + LINE_STEER_SLEW;
    if (delta < -LINE_STEER_SLEW) steer = last_steer - LINE_STEER_SLEW;
    last_steer = steer;

    return (float)steer;
}

/**
 * LineSensor_IsValid — 是否检测到有效黑线
 */
bool LineSensor_IsValid(void)
{
    return has_line;
}

/**
 * LineSensor_IsLost — 是否丢线（连续多次无黑线）
 */
bool LineSensor_IsLost(void)
{
    return lost_count > LINE_LOST_TICKS;
}

/**
 * LineSensor_IsEndpoint — 是否到达弧线终点
 */
bool LineSensor_IsEndpoint(void)
{
    return false;
}
