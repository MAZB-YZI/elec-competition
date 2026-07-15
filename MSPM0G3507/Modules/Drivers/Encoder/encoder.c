#include "encoder.h"

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
    speed.left = count_to_m(l - last_left) / dt_s;
    speed.right = count_to_m(r - last_right) / dt_s;
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
        count_to_m(left_count - zero_left),
        count_to_m(right_count - zero_right)
    };
    return v;
}

void Encoder_ResetDistance(void)
{
    zero_left = left_count;
    zero_right = right_count;
}
