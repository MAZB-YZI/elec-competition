#include "control.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "jy61p.h"
#include <math.h>

/* ========== 速度环 PI ========== */
typedef struct { float kp, ki, integral; } SpeedPI;
static SpeedPI left_pi = {80.0f, 20.0f, 0.0f};
static SpeedPI right_pi = {80.0f, 20.0f, 0.0f};

/* ========== 航向环 PD ========== */
static float heading_kp = 5.0f;
static float heading_kd = 0.2f;
static float heading_limit = 500.0f;

/* ========== 循迹环（固定 PWM + 转向，PD 控制在 line_sensor.c 内部） ========== */
static int16_t line_base_pwm = 600;     /* 基准 PWM */

/* ========== 内部状态 ========== */
static MotionState state;
static float target_distance, target_speed, target_yaw, last_heading_error;

/* ========== 辅助函数 ========== */
static float clamp(float value, float limit)
{ return value > limit ? limit : (value < -limit ? -limit : value); }

static float forward_only(float value)
{ return value < 0.0f ? 0.0f : value; }

static float speed_pi(SpeedPI *pi, float error)
{
    pi->integral = clamp(pi->integral + error * 0.005f, 10.0f);
    return clamp(pi->kp * error + pi->ki * pi->integral, 4000.0f);
}

static void reset_controllers(void)
{
    left_pi.integral = 0.0f;
    right_pi.integral = 0.0f;
    last_heading_error = 0.0f;
}

/* ========== 参数调节接口 ========== */
void Motion_SetSpeedGains(float kp, float ki)
{
    left_pi.kp = kp; left_pi.ki = ki;
    right_pi.kp = kp; right_pi.ki = ki;
}

void Motion_SetHeadingGains(float kp, float kd, float limit)
{
    heading_kp = kp;
    heading_kd = kd;
    heading_limit = limit;
}

void Motion_SetLineBasePWM(int16_t base_pwm)
{
    line_base_pwm = base_pwm;
}

/* ========== 运动控制接口 ========== */
void Motion_Init(void) { state = MOTION_IDLE; reset_controllers(); Motor_Stop(); }

void Motion_DriveDistance(float metres, float speed_mps)
{
    Encoder_ResetDistance();
    reset_controllers();
    target_distance = fabsf(metres);
    target_speed = fabsf(speed_mps); /* H题只允许前进 */
    target_yaw = JY61P_GetYaw();
    state = MOTION_DISTANCE;
}

void Motion_DriveHeading(float metres, float speed_mps, float yaw_deg)
{ Motion_DriveDistance(metres, speed_mps); target_yaw = yaw_deg; state = MOTION_HEADING; }

void Motion_FollowLine(float speed_mps)
{ Encoder_ResetDistance(); reset_controllers(); target_speed = fabsf(speed_mps); target_distance = 0.0f; state = MOTION_LINE; }

void Motion_FollowLineDistance(float speed_mps, float max_metres)
{ Encoder_ResetDistance(); reset_controllers(); target_speed = fabsf(speed_mps); target_distance = fabsf(max_metres); state = MOTION_LINE; }

void Motion_Stop(void) { Motor_Stop(); state = MOTION_IDLE; }

void Motion_Update10ms(void)
{
    EncoderPair velocity = Encoder_GetSpeed();
    EncoderPair travelled = Encoder_GetDistance();
    float steering;

    if (state == MOTION_IDLE || state == MOTION_DONE || state == MOTION_FAULT) {
        Motor_Stop();
        return;
    }

    if (state == MOTION_LINE) {
        /* 循迹：GetError() 内部已完成 PD+死区+限速，直接返回 steer */
        int16_t steer = (int16_t)LineSensor_GetError();
        // TODO: 调参阶段暂时注释掉停止条件，让车一直跑
        // if (LineSensor_IsEndpoint()) { Motor_Stop(); state = MOTION_DONE; return; }
        // if (!LineSensor_IsValid() || LineSensor_IsLost()) { Motor_Stop(); state = MOTION_FAULT; return; }
        // if (target_distance > 0.0f) {
        //     float avg_dist = (fabsf(travelled.left) + fabsf(travelled.right)) * 0.5f;
        //     if (avg_dist >= target_distance) { Motor_Stop(); state = MOTION_DONE; return; }
        // }
        /* 丢线时保持上次转向继续走 */
        if (!LineSensor_IsValid() || LineSensor_IsLost()) {
            // 不停车，保持当前转向继续
        }
        /* H题限制：内侧轮不能反转 */
        if (steer > line_base_pwm) steer = line_base_pwm;
        if (steer < -line_base_pwm) steer = -line_base_pwm;
        Motor_SetPWM((int32_t)(line_base_pwm - steer),
                     (int32_t)(line_base_pwm + steer));
        return;
    } else {
        /* 直线/对角线：速度 PI + 航向 PD */
        float heading_error = target_yaw - JY61P_GetYaw();
        steering = clamp(heading_kp * heading_error +
            heading_kd * (heading_error - last_heading_error) / 0.005f, heading_limit);
        last_heading_error = heading_error;
        if ((fabsf(travelled.left) + fabsf(travelled.right)) * 0.5f >= target_distance) {
            Motor_Stop(); state = MOTION_DONE; return;
        }
    }

    Motor_SetPWM((int32_t)forward_only(speed_pi(&left_pi, target_speed - velocity.left) - steering),
                 (int32_t)forward_only(speed_pi(&right_pi, target_speed - velocity.right) + steering));
}

MotionState Motion_GetState(void) { return state; }
bool Motion_IsComplete(void) { return state == MOTION_DONE; }
bool Motion_HasFault(void) { return state == MOTION_FAULT; }
