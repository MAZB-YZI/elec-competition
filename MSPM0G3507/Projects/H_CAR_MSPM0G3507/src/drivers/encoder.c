/**
 * encoder.c — H_CAR 编码器适配层
 *
 * 基于共享模块 Modules/Drivers/Encoder，加方向归一化：
 *   前进时 left > 0, right > 0
 *
 * 方向系数说明：
 *   左编码器 A=PA25, B=PA14  → 前进时 B 高 → count++ → 正确，系数 +1
 *   右编码器 A=PA26, B=PA27  → 前进时 B 低 → count-- → 取反，系数 -1
 *
 * 如果实际装车后方向不对，只改这两个系数即可。
 */

#include "encoder.h"

/* 方向归一化系数：前进时两个速度都 > 0 */
#define HCAR_LEFT_ENCODER_DIR   -1
#define HCAR_RIGHT_ENCODER_DIR   1

/* 引入共享模块实现（不直接 #include .c，由 CCS 工程编译） */
/* 注意：本文件和共享模块 encoder.c 不能同时编译，否则符号冲突。
 * 当前用 bridge 方式：本文件替代共享模块的 encoder.c。
 * 如果改为 CCS linked source，删除本文件，在 SysConfig 中统一引脚名。 */

/* ========== 共享模块内部变量（从 encoder.c 复制） ========== */
#include <math.h>

#define ENCODER_COUNTS_PER_REV 260.0f
#define WHEEL_CIRCUMFERENCE_M  0.2104867f

static volatile int32_t left_count;
static volatile int32_t right_count;
static int32_t last_left;
static int32_t last_right;
static int32_t zero_left;
static int32_t zero_right;
static EncoderPair speed;

static float count_to_m(int32_t count)
{
    return count * WHEEL_CIRCUMFERENCE_M / ENCODER_COUNTS_PER_REV;
}

void Encoder_Init(void)
{
    left_count = 0;
    right_count = 0;
    last_left = 0;
    last_right = 0;
    zero_left = 0;
    zero_right = 0;
    speed.left = 0.0f;
    speed.right = 0.0f;
}

void Encoder_OnLeftAEdge(void)
{
    left_count += EncoderHal_ReadPhaseB(true) ? -1 : 1;
}

void Encoder_OnRightAEdge(void)
{
    right_count += EncoderHal_ReadPhaseB(false) ? -1 : 1;
}

void Encoder_Update(float dt_s)
{
    int32_t l;
    int32_t r;

    if (dt_s <= 0.0f) {
        return;
    }

    l = left_count;
    r = right_count;
    /* 方向归一化：乘以方向系数 */
    speed.left = count_to_m(l - last_left) * (float)HCAR_LEFT_ENCODER_DIR / dt_s;
    speed.right = count_to_m(r - last_right) * (float)HCAR_RIGHT_ENCODER_DIR / dt_s;
    last_left = l;
    last_right = r;
}

EncoderCounts Encoder_GetCount(void)
{
    EncoderCounts v = { left_count, right_count };
    return v;
}

EncoderPair Encoder_GetSpeed(void)
{
    return speed;
}

EncoderPair Encoder_GetDistance(void)
{
    EncoderPair v = {
        count_to_m(left_count - zero_left) * (float)HCAR_LEFT_ENCODER_DIR,
        count_to_m(right_count - zero_right) * (float)HCAR_RIGHT_ENCODER_DIR
    };
    return v;
}

void Encoder_ResetDistance(void)
{
    zero_left = left_count;
    zero_right = right_count;
}
