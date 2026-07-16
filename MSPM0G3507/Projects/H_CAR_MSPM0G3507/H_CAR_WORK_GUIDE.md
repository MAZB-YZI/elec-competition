# H_CAR_MSPM0G3507 work guide

Goal: solve 2024 H problem "automatic driving car" using this project, while reusing `D:/TI/Projects/elec-competition/MSPM0G3507/Modules` instead of duplicating drivers.

## Current project status

This project already has the right high-level shape:

| Layer | Existing files | Meaning |
|---|---|---|
| Route FSM | `src/control/route_fsm.c`, `include/route_fsm.h` | Chooses test 1/2/3/4 and segment order. |
| Motion control | `src/control/control.c`, `include/control.h` | Distance driving, heading driving, line following, 10 ms update. |
| Board HAL | `src/drivers/hcar_hal.c`, `include/hcar_hal.h` | Binds shared modules to this board's SysConfig macros. |
| Reusable bridges | `src/drivers/*.c`, `include/*.h` | Some files forward to `Modules` or adapt hardware. |
| Test entry | `main.c`, `empty.c` | Current bring-up/test entry points. |

The project is not finished as a contest solution yet. Treat it as the correct place to continue development.

## The route for the H problem

Problem geometry:

- A-B straight: 1.00 m
- C-D straight: 1.00 m
- A-C diagonal: about 1.280625 m
- B-D diagonal: about 1.280625 m
- Each half-circle radius: 0.40 m
- Half-circle arc length: about 1.2566 m

Requirement mapping:

| Requirement | Route state sequence |
|---|---|
| 1 | A -> B straight, stop and beep/light |
| 2 | A -> B straight, B -> C arc, C -> D straight, D -> A arc |
| 3 | A -> C diagonal, C -> B arc, B -> D diagonal, D -> A arc |
| 4 | Requirement 3 route, repeat 4 laps, then stop |

## Recommended implementation order

### Step 1: keep a safe entry

Keep `empty.c` or a simple LED/buzzer test build available. Do not jump straight to full route code after hardware changes.

### Step 2: make motor PWM real

Open `src/drivers/hcar_hal.c` and finish `HCarHal_SetMotorDuty()` after SysConfig has PA12/PA13 PWM macros. Until this works, route code cannot move reliably.

Expected test:

```text
left PWM 600 -> left wheel moves forward
right PWM 600 -> right wheel moves forward
Motor_Stop -> both stop
```

### Step 3: normalize encoders

Use the existing encoder module. Confirm by hand:

```text
left wheel forward  -> left count increases
right wheel forward -> right count increases
```

If one side is reversed, fix it in the board adapter or with direction constants. Do not fix this inside route_fsm.

### Step 4: distance straight test

Use `Motion_DriveDistance(1.0f, speed)` for A -> B.

Start slow:

```c
STRAIGHT_SPEED = 0.20f to 0.30f
```

Watch:

```text
average distance = (abs(left) + abs(right)) / 2
heading error from IMU
final stop position at B
```

### Step 5: yaw hold for straight/diagonal

Straight sections with no black line should use encoder distance plus yaw hold.

Use:

```c
Motion_DriveHeading(distance, speed, target_yaw)
```

For first version, record yaw at segment start and hold it. Later, use known route heading offsets if needed.

### Step 6: line sensor adapter for arcs

Replace `src/drivers/line_sensor_stub.c` with a real adapter around the gray sensor module.

The route control expects:

```c
LineSensor_GetError();     // negative/positive line offset
LineSensor_IsValid();      // line visible
LineSensor_IsEndpoint();   // point or arc end detected
LineSensor_IsLost();       // line lost long enough to stop
```

For the H problem, endpoint detection can start simple:

```text
arc distance >= 1.20 m, then accept endpoint
```

Then refine with gray pattern if needed.

### Step 7: tune half-circle following

Use `Motion_FollowLine(LINE_SPEED)` only on black arcs.

Start with:

```c
LINE_SPEED = 0.18f to 0.22f
```

Tune line steering in `control.c`:

```c
steering = line_kp * LineSensor_GetError()
```

The car projection must stay on the arc, so stability matters more than speed at first.

### Step 8: route FSM

`route_fsm.c` already has the right rough structure:

```c
test1 = straight
test2 = straight, line, straight, line
test3 = diagonal, line, diagonal, line
test4 = test3 repeated 4 laps
```

After each segment completes:

```c
Buzzer_Beep(120);
point_event = true;
```

Add LED prompt with the buzzer if the current hardware supports it.

## Parameters to tune first

In `src/control/route_fsm.c`:

```c
#define STRAIGHT_M       1.0000f
#define DIAGONAL_M       1.280625f
#define STRAIGHT_SPEED   0.20f to 0.35f
#define DIAGONAL_SPEED   0.18f to 0.30f
#define LINE_SPEED       0.16f to 0.24f
```

In `src/control/control.c`:

```c
left/right speed PI gains
heading hold Kp/Kd
line steering Kp and steering limit
```

Tune in this order:

1. Encoder direction and distance scale.
2. Open-loop motor symmetry.
3. Speed PI.
4. Yaw hold straight line.
5. Arc line following.
6. Route state transitions.

## What not to do

- Do not copy `Liner_Car0_Rebuilt/main.c` into this project as the final solution. Its L-turn logic is for a different track shape.
- Do not add duplicated PID/motor/encoder/OLED folders under this project.
- Do not hand-edit `.syscfg`, `.project`, `.cproject`, or generated `Debug/syscfg` files.
- Do not enable speed loop before encoder signs are correct.
- Do not use reverse motion for path correction; the problem says the car may only move forward.

## Practical next coding target

The next real code target should be one of these, in order:

1. Finish `HCarHal_SetMotorDuty()` and verify PWM output.
2. Add encoder direction constants and verify forward-positive counts.
3. Make a test mode that runs exactly 1.00 m and stops with buzzer/LED.
4. Replace `line_sensor_stub.c` with real gray sensor logic.
5. Tune `Motion_FollowLine()` on one half-circle.
