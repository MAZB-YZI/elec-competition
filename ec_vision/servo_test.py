# -*- coding: utf-8 -*-
"""
servo_test.py — PWM 舵机云台隔离测试(仅 Drv=0 备用方案用, F32C 用 f32c_test.py)
接线见 GIMBAL_WIRING.md 附录。流程: 回中 -> YAW ±30° 慢扫 -> 回中 -> PITCH ±30° 慢扫 -> 回中。
运行时记录: 占空比增大云台往哪转 —— 决定 GIMB 页 InvX/InvY。
某轴不动: 查该信号线; 两轴都不动: 查外部 5V 与共地; 一动就重启: 供电不足, 换独立电源。
"""
from maix import pwm, pinmap, err, sys, time

FREQ = 50
CENTER = 7.5                    # 1.5ms = 90°
DEG = 10.0 / 180.0              # 每度占空比: (12.5-2.5)/180

if sys.device_id() == "maixcam2":
    PIN_PITCH, ID_PITCH, PIN_YAW, ID_YAW = "A30", 6, "A31", 7
else:
    PIN_PITCH, ID_PITCH, PIN_YAW, ID_YAW = "A18", 6, "A19", 7

err.check_raise(pinmap.set_pin_function(PIN_PITCH, "PWM%d" % ID_PITCH), "pinmap pitch")
err.check_raise(pinmap.set_pin_function(PIN_YAW, "PWM%d" % ID_YAW), "pinmap yaw")
sv_pitch = pwm.PWM(ID_PITCH, freq=FREQ, duty=CENTER, enable=True)
sv_yaw = pwm.PWM(ID_YAW, freq=FREQ, duty=CENTER, enable=True)
print("回中 (duty=%.1f%% = 90°)  pitch=%s  yaw=%s" % (CENTER, PIN_PITCH, PIN_YAW))
time.sleep_ms(1500)


def sweep(sv, name):
    print(name, "扫摆 ±30°, 记录: 占空比增大往哪转")
    for d in list(range(0, 31, 2)) + list(range(30, -31, -2)) + list(range(-30, 1, 2)):
        sv.duty(CENTER + d * DEG)
        time.sleep_ms(60)
    print(name, "回中")


sweep(sv_yaw, "[YAW %s]" % PIN_YAW)
time.sleep_ms(800)
sweep(sv_pitch, "[PITCH %s]" % PIN_PITCH)
print("完成")
