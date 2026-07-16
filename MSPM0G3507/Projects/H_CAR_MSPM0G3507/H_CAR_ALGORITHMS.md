# H_CAR algorithm ideas to absorb from Liner_Car0_Rebuilt

This file records control ideas worth moving into `Projects/H_CAR_MSPM0G3507`. These are algorithms and debug workflows, not extra copied modules.

## 1. Encoder direction normalization

Where to implement:

- Prefer: `src/drivers/hcar_hal.c` or the H_CAR encoder adapter.
- Alternative: `Modules/Drivers/Encoder/encoder.c` if we decide direction signs are a reusable board configuration.

Goal:

```text
When the car moves forward:
left speed  > 0
right speed > 0
```

Suggested parameters:

```c
#define HCAR_LEFT_ENCODER_DIR    1
#define HCAR_RIGHT_ENCODER_DIR  -1
```

Use them when converting raw count delta into speed or when updating raw counts. Do this before enabling speed PI/PID.

Acceptance test:

```text
Lift the car.
Turn left wheel forward  -> left count increases.
Turn right wheel forward -> right count increases.
Run both motors forward  -> both speeds are positive.
```

Why it matters:

Speed loop, distance stop, lap counting, and D = left - right are all wrong if one wheel's encoder sign is reversed.

## 2. Open-loop encoder test before speed loop

Where to implement:

- `main.c` test mode, or a small bring-up function in `src/app/`.

Idea:

Keep speed loop disabled at first. Send the same PWM to both motors and only observe encoder values.

Suggested parameters:

```c
#define HCAR_SPEED_LOOP_ENABLE 0
#define HCAR_TEST_PWM          600
```

OLED/debug output:

```text
L:+xxxx R:+xxxx
D:+xxxx
```

Meaning:

```c
D = left_speed - right_speed;
```

Acceptance test:

```text
If right wheel is visibly faster, R should be larger and D should be negative.
If D has the wrong sign, fix encoder mapping/sign before PID.
```

## 3. OLED debug page for motion bring-up

Where to implement:

- Project debug display code in `main.c` or `src/app/hcar_app.c`.
- Do not make route logic depend on OLED.

Recommended page:

```text
Line 1: current route/motion state
Line 2: L speed, R speed
Line 3: D speed difference, distance
Line 4: yaw, accumulated angle/lap
```

Useful variables:

```c
left_speed
right_speed
speed_diff = left_speed - right_speed
avg_distance = (fabs(left_dist) + fabs(right_dist)) * 0.5f
yaw_deg
accumulated_yaw_deg
lap_count
```

Why it matters:

The H problem is hard to tune blind. This page tells whether failure is from encoder, yaw, line tracking, or route state switching.

## 4. Yaw wrap-safe difference

Where to implement:

- `src/control/control.c` or a small utility file if reused by route/debug code.

Function:

```c
static float yaw_diff(float current, float reference)
{
    float diff = current - reference;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}
```

Use cases:

1. Heading hold on straight/diagonal segments.
2. Accumulated angle/lap debug.
3. Detecting large turns or route orientation changes.

Do not use raw `current - reference` for yaw across +180/-180.

## 5. Accumulated yaw angle

Where to implement:

- `src/control/control.c` for motion debug.
- Or `src/app` if it is only display telemetry.

Algorithm:

```c
float delta = yaw_diff(yaw_now, yaw_last);
yaw_last = yaw_now;
if (delta < 0.0f) delta = -delta;
accumulated_yaw += delta;
```

Debug conversion:

```c
turns = accumulated_yaw / 360.0f;
```

Use in H_CAR:

This is mainly a diagnostic value. The route FSM should still use segment completion by distance/line endpoint. Do not rely only on accumulated yaw to decide A/B/C/D points.

## 6. Distance-triggered segment completion

Where to implement:

- `src/control/control.c` for `Motion_DriveDistance()`.
- `src/control/route_fsm.c` for per-segment distances.

Liner idea:

`TURN_FORWARD_PULSES` means "continue until encoder distance reaches a threshold".

H_CAR version:

```c
#define HCAR_STRAIGHT_M   1.0000f
#define HCAR_DIAGONAL_M   1.280625f
#define HCAR_ARC_LENGTH_M 1.2566f
```

For straight/diagonal:

```c
avg_distance = (fabs(left_dist) + fabs(right_dist)) * 0.5f;
if (avg_distance >= target_distance) segment_done = true;
```

For arcs:

Use line following, but keep arc length as a fallback:

```c
if (line_endpoint_detected || avg_distance >= HCAR_ARC_LENGTH_M * 0.95f) segment_done = true;
```

## 7. Straight segment = distance + yaw hold

Where to implement:

- `src/control/control.c`, inside `Motion_DriveHeading()` / `Motion_Update10ms()`.

Control idea:

```text
base speed from speed PI
steering correction from yaw error
left command  = base - steering
right command = base + steering
```

Suggested first parameters:

```c
#define HCAR_HEADING_KP       20.0f
#define HCAR_HEADING_KD        0.8f
#define HCAR_HEADING_STEER_LIM 1200.0f
```

Start conservative. A slow straight line that stops correctly is worth more than a fast one that misses B/D.

## 8. Arc segment = gray line following + distance fallback

Where to implement:

- Real adapter: `src/drivers/line_sensor_*.c` or replace `line_sensor_stub.c`.
- Control: `src/control/control.c` in `MOTION_LINE`.

Interface needed by H_CAR:

```c
float LineSensor_GetError(void);
bool LineSensor_IsValid(void);
bool LineSensor_IsLost(void);
bool LineSensor_IsEndpoint(void);
```

First version of endpoint:

```text
Line valid + arc distance nearly reached.
```

Later version:

```text
Use gray sensor pattern changes near arc top point, if reliable.
```

Suggested first parameters:

```c
#define HCAR_LINE_SPEED       0.16f to 0.22f
#define HCAR_LINE_KP          tune from low to high
#define HCAR_LINE_STEER_LIM   1200.0f to 1600.0f
#define HCAR_LINE_LOST_TICKS  10 to 30 control ticks
```

## 9. Speed loop enable gate

Where to implement:

- `src/control/control.c`.

Idea:

Keep a compile-time or runtime switch:

```c
#define HCAR_SPEED_LOOP_ENABLE 0
```

When disabled:

```text
Use fixed open-loop PWM/speed command for motor symmetry tests.
```

When enabled:

```text
Use speed PI/PID for straight and arc movement.
```

Enable only after:

1. Motor directions are correct.
2. Encoder signs are correct.
3. Speed readings are stable.
4. PWM output works on both wheels.

## 10. Route state machine with point prompt

Where to implement:

- `src/control/route_fsm.c`.

Keep this route structure:

```text
Requirement 1:
A_B_STRAIGHT -> STOP

Requirement 2:
A_B_STRAIGHT -> B_C_ARC -> C_D_STRAIGHT -> D_A_ARC -> STOP

Requirement 3:
A_C_DIAGONAL -> C_B_ARC -> B_D_DIAGONAL -> D_A_ARC -> STOP

Requirement 4:
Requirement 3 repeated 4 laps -> STOP
```

On every segment completion:

```c
Buzzer_Beep(120);
StatusLed_Pulse();
point_event = true;
```

Do not detect A/B/C/D by text or external marks. They do not exist on the real field. Infer point transitions from the route state and segment completion.

## 11. Safety stops

Where to implement:

- `src/control/control.c`.
- `src/control/route_fsm.c`.

Stop conditions:

```text
line lost too long during arc
encoder speed impossible or zero while PWM high
IMU offline or yaw invalid during straight
segment timeout exceeded
route abort requested
```

Suggested first timeouts:

```c
A_B 1.0 m straight: 8 to 12 s while tuning
Arc half circle:    10 to 15 s while tuning
Full test 2:        under 30 s after tuning
Full test 3:        under 40 s after tuning
```

## 12. Parameter table to add later

Centralize these in `route_fsm.c` or a future `hcar_params.h`:

```c
#define HCAR_STRAIGHT_M       1.0000f
#define HCAR_DIAGONAL_M       1.280625f
#define HCAR_ARC_LENGTH_M     1.2566f
#define HCAR_STRAIGHT_SPEED   0.20f
#define HCAR_DIAGONAL_SPEED   0.18f
#define HCAR_LINE_SPEED       0.16f
#define HCAR_POINT_BEEP_MS    120U
#define HCAR_TEST4_LAPS       4U
```

Tune speed only after the car can finish the route slowly and reliably.

## Absorption plan

Do not implement everything at once. Move these ideas into code in this order:

1. Encoder direction normalization.
2. Open-loop OLED speed/difference page.
3. Distance-triggered A -> B straight stop.
4. Yaw hold for straight/diagonal.
5. Gray line error/valid/lost/endpoint interface.
6. Arc line following with distance fallback.
7. Route FSM point prompt and lap count.
8. Enable speed loop and raise speed.
