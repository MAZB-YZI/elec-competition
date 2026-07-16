/**
 * line_sensor.c — H_CAR 灰度循迹适配层
 *
 * 基于 Modules/Drivers/GraySensor 的 8 路灰度传感器
 * 实现 LineSensor 接口供 control.c 使用
 *
 * 引脚: PB6(CLK 输出), PB7(DAT 输入) — SysConfig 配置
 * 协议: CLK 发 8 个脉冲, DAT 线串行读出 8 路灰度值
 *       0=黑线, 1=白底
 */

#include "line_sensor.h"
#include "gray_sensor.h"
#include <math.h>

/* ========== 参数 ========== */
#define LINE_VALID_MIN     1      /* 至少检测到1路黑线才算有效 */
#define LINE_LOST_THRESHOLD 0     /* 0路黑线 = 丢线 */
#define ENDPOINT_DISTANCE  1.10f  /* 弧线终点兜底距离(米)，暂时不用 */

/* ========== 内部状态 ========== */
static uint8_t last_raw = 0U;
static bool has_line = false;
static uint32_t lost_count = 0U;
static uint32_t valid_count = 0U;

/* ========== LineSensor 接口实现 ========== */

/**
 * LineSensor_GetError — 返回线位置误差
 *
 * 加权计算: 外侧权重高, 内侧权重低
 * 返回值: 负=线在左边, 正=线在右边, 0=居中
 *
 * 参考 Liner_Car0_Rebuilt 的加权公式:
 * pos = 200*s[0] + 140*s[1] + 75*s[2] + 40*s[3]
 *     - 40*s[4] - 75*s[5] - 140*s[6] - 200*s[7]
 */
float LineSensor_GetError(void)
{
    uint8_t raw = GraySensor_Read();
    last_raw = raw;

    /* 加权位置计算: 外侧权重高 */
    int16_t pos = ( 200) * ((raw >> 0) & 1)
                + ( 140) * ((raw >> 1) & 1)
                + (  75) * ((raw >> 2) & 1)
                + (  40) * ((raw >> 3) & 1)
                + ( -40) * ((raw >> 4) & 1)
                + ( -75) * ((raw >> 5) & 1)
                + (-140) * ((raw >> 6) & 1)
                + (-200) * ((raw >> 7) & 1);

    /* 归一化到 ±1.0 范围 */
    return (float)pos / 200.0f;
}

/**
 * LineSensor_IsValid — 是否检测到有效黑线
 *
 * 有效: 至少 LINE_VALID_MIN 路检测到黑线, 且非全黑
 */
bool LineSensor_IsValid(void)
{
    uint8_t raw = last_raw;
    uint8_t black_count = 0U;

    for (uint8_t i = 0; i < 8; i++) {
        if (!((raw >> i) & 1)) {  /* 0 = 黑线 */
            black_count++;
        }
    }

    has_line = (black_count >= LINE_VALID_MIN) && (black_count < 8U);
    if (has_line) {
        valid_count++;
        lost_count = 0U;
    }
    return has_line;
}

/**
 * LineSensor_IsLost — 是否丢线
 *
 * 丢线: 连续 N 次读取都没有检测到黑线
 */
bool LineSensor_IsLost(void)
{
    if (!has_line) {
        lost_count++;
    }
    return lost_count > 10U;  /* 连续10次(约100ms)无线 = 丢线 */
}

/**
 * LineSensor_IsEndpoint — 是否到达弧线终点
 *
 * 当前版本: 不使用终点检测, 由编码器距离兜底
 * 后续可加: 检测线形变化(如从弧线变成直线)
 */
bool LineSensor_IsEndpoint(void)
{
    return false;
}
