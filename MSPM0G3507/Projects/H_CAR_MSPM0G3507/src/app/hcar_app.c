#include "hcar_app.h"
#include "buzzer.h"
#include "control.h"
#include "encoder.h"
#include "hcar_hal.h"
#include "motor.h"
#include "mpu6050.h"

static volatile uint32_t milliseconds;
static uint32_t processed_ms;
static HCarAppState app_state;
static HCarSelfTest self_test;
static uint32_t test_deadline;
static uint32_t led_deadline;
static RouteState previous_route_state;

static void fail_safe(void)
{
    Motor_Stop();
    Buzzer_Stop();
    HCarHal_SetStatusLed(false);
    self_test = HCAR_TEST_NONE;
    app_state = HCAR_APP_FAULT;
}

bool HCarApp_Init(void)
{
    app_state = HCAR_APP_SAFE;
    milliseconds = 0;
    processed_ms = 0;
    self_test = HCAR_TEST_NONE;
    if (!HCarHal_Init()) { fail_safe(); return false; }
    Motor_Init();
    Encoder_Init();
    Buzzer_Init();
    Motion_Init();
    Route_Init();
    HCarHal_SetStatusLed(false);
    if (!MPU6050_Init() || !MPU6050_CalibrateGyro(500)) {
        fail_safe();
        return false;
    }
    MPU6050_ResetYaw();
    previous_route_state = Route_GetState();
    app_state = HCAR_APP_READY;
    return true;
}

void HCarApp_Tick1msISR(void)
{
    ++milliseconds;
    Buzzer_Update1ms();
}

static void update_self_test(void)
{
    if (self_test == HCAR_TEST_NONE) return;
    if ((int32_t)(milliseconds - test_deadline) < 0) return;
    switch (self_test) {
    case HCAR_TEST_MOTOR:
        Motor_Stop();
        self_test = HCAR_TEST_NONE;
        app_state = HCAR_APP_READY;
        break;
    case HCAR_TEST_ENCODER:
        Motor_Stop();
        Encoder_Update(0.5f);
        self_test = HCAR_TEST_NONE;
        app_state = HCAR_APP_READY;
        break;
    case HCAR_TEST_MPU6050:
    case HCAR_TEST_BUZZER:
        self_test = HCAR_TEST_NONE;
        app_state = HCAR_APP_READY;
        break;
    default:
        fail_safe();
        break;
    }
}

void HCarApp_RunPending(void)
{
    while (processed_ms != milliseconds) {
        ++processed_ms;
        if (led_deadline != 0U && (int32_t)(processed_ms - led_deadline) >= 0) {
            HCarHal_SetStatusLed(false);
            led_deadline = 0U;
        }
        if ((processed_ms % 5U) == 0U && !MPU6050_Update(0.005f)) {
            fail_safe();
            return;
        }
        if ((processed_ms % 10U) == 0U) {
            Encoder_Update(0.01f);
            if (app_state == HCAR_APP_RUNNING && self_test == HCAR_TEST_NONE) {
                Motion_Update10ms();
                Route_Update();
                if (Route_ConsumePointEvent()) {
                    HCarHal_SetStatusLed(true);
                    led_deadline = processed_ms + 120U;
                }
                if (Route_GetState() != previous_route_state) {
                    previous_route_state = Route_GetState();
                    HCarHal_SetStatusLed(previous_route_state == ROUTE_COMPLETE);
                }
                if (previous_route_state == ROUTE_COMPLETE) app_state = HCAR_APP_READY;
                else if (previous_route_state == ROUTE_ERROR) fail_safe();
            }
            update_self_test();
        }
    }
}

bool HCarApp_StartRoute(RouteTest test)
{
    if (app_state != HCAR_APP_READY || self_test != HCAR_TEST_NONE) return false;
    HCarHal_SetStatusLed(false);
    MPU6050_ResetYaw();
    if (!Route_Start(test)) return false;
    previous_route_state = ROUTE_RUNNING;
    app_state = HCAR_APP_RUNNING;
    return true;
}

bool HCarApp_StartSelfTest(HCarSelfTest test)
{
    uint16_t duty;
    if (app_state != HCAR_APP_READY || test == HCAR_TEST_NONE) return false;
    duty = HCarHal_GetMotorPeriod() / 8U;
    self_test = test;
    app_state = HCAR_APP_RUNNING;
    switch (test) {
    case HCAR_TEST_MOTOR:
        Motor_SetPWM(duty, duty);
        test_deadline = milliseconds + 500U;
        break;
    case HCAR_TEST_ENCODER:
        Encoder_ResetDistance();
        Motor_SetPWM(duty, duty);
        test_deadline = milliseconds + 500U;
        break;
    case HCAR_TEST_MPU6050:
        MPU6050_ResetYaw();
        test_deadline = milliseconds + 2000U;
        break;
    case HCAR_TEST_BUZZER:
        Buzzer_Beep(200U);
        test_deadline = milliseconds + 250U;
        break;
    default:
        fail_safe();
        return false;
    }
    return true;
}

void HCarApp_EmergencyStop(void)
{
    Route_Abort();
    Motor_Brake();
    Buzzer_Stop();
    HCarHal_SetStatusLed(false);
    self_test = HCAR_TEST_NONE;
    app_state = HCAR_APP_FAULT;
}

HCarAppState HCarApp_GetState(void) { return app_state; }
HCarSelfTest HCarApp_GetSelfTest(void) { return self_test; }
uint32_t HCarApp_GetMilliseconds(void) { return milliseconds; }
