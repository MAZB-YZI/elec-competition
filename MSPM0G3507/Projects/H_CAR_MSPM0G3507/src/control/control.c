#include "control.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "mpu6050.h"
#include <math.h>

typedef struct { float kp, ki, integral; } SpeedPI;
static SpeedPI left_pi = {1200.0f, 400.0f, 0.0f};
static SpeedPI right_pi = {1200.0f, 400.0f, 0.0f};
static MotionState state;
static float target_distance, target_speed, target_yaw, last_heading_error;

static float clamp(float value, float limit)
{ return value > limit ? limit : (value < -limit ? -limit : value); }

static float forward_only(float value)
{ return value < 0.0f ? 0.0f : value; }

static float speed_pi(SpeedPI *pi, float error)
{
    pi->integral = clamp(pi->integral + error * 0.01f, 10.0f);
    return clamp(pi->kp * error + pi->ki * pi->integral, 4000.0f);
}

static void reset_controllers(void)
{
    left_pi.integral = 0.0f;
    right_pi.integral = 0.0f;
    last_heading_error = 0.0f;
}

void Motion_Init(void) { state = MOTION_IDLE; reset_controllers(); Motor_Stop(); }

void Motion_DriveDistance(float metres, float speed_mps)
{
    Encoder_ResetDistance();
    reset_controllers();
    target_distance = fabsf(metres);
    target_speed = fabsf(speed_mps); /* H题只允许前进 */
    target_yaw = MPU6050_GetYaw();
    state = MOTION_DISTANCE;
}

void Motion_DriveHeading(float metres, float speed_mps, float yaw_deg)
{ Motion_DriveDistance(metres, speed_mps); target_yaw = yaw_deg; state = MOTION_HEADING; }

void Motion_FollowLine(float speed_mps)
{ reset_controllers(); target_speed = fabsf(speed_mps); state = MOTION_LINE; }

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
        if (LineSensor_IsEndpoint()) { Motor_Stop(); state = MOTION_DONE; return; }
        if (!LineSensor_IsValid() || LineSensor_IsLost()) { Motor_Stop(); state = MOTION_FAULT; return; }
        steering = clamp(900.0f * LineSensor_GetError(), 1400.0f);
    } else {
        float heading_error = target_yaw - MPU6050_GetYaw();
        steering = clamp(25.0f * heading_error +
            (heading_error - last_heading_error) / 0.01f, 1200.0f);
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
