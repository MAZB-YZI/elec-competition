# -*- coding: utf-8 -*-
"""
f32c_test.py — F32C 云台电机隔离测试(不跑视觉框架, 接线/协议一次验清)
用法: 电机 12V 上电 ≥5 秒后运行本脚本。全程打印收发帧 hex, 出问题一眼定位。
流程: 失能->清零->使能->位置模式->限速 -> 读电压/角度反馈 -> YAW ±20° -> PITCH ±10° -> 回零 -> 失能
判读:
  - "RX(raw)" 一直是空 => 收不到反馈: 对调两根信号线; 仍不行查共地/波特率/电机ID
  - 电压反馈 ≈ 你的供电电压 => 总线双向通, 协议对
  - 扫摆时记录: 角度增大云台往哪转 => 决定 GIMB 页 InvX/InvY
  - 只有一个电机动 => 另一个 ID 不对(需厂商软件确认 YAW=1, PITCH=2)
"""
from maix import time
import f32c
from f32c import F32CGimbal, frame, CMD_FEEDBACK, CMD_MULTI_POS, CMD_DISABLE
from f32c import FB_VOLTAGE, FB_MULTI_ANGLE, _i32be


def hexs(b):
    return " ".join("%02X" % x for x in (b or b""))


class F32CTest(F32CGimbal):
    """打印所有收发字节的调试版。"""
    def _tx(self, addr, cmd, data=b"", gap_ms=0):
        f = frame(addr, cmd, data)
        print("TX ->", hexs(f))
        self.u.write(f)
        if gap_ms:
            time.sleep_ms(gap_ms)

    def _read_fb(self, addr, fb_type, timeout_ms=500):
        time.sleep_ms(f32c.INTER_FRAME_MS)      # 同 f32c.py: 读反馈前必须留帧间隔
        self._tx(addr, CMD_FEEDBACK, bytes([fb_type]))
        t0 = time.ticks_ms()
        while time.ticks_ms() - t0 < timeout_ms:
            try:
                data = self.u.read()
            except Exception:
                data = None
            if data:
                print("RX(raw) <-", hexs(data))
            for a, t, v in self.fb.feed(data):
                print("   解析: addr=%d type=%d value=%d" % (a, t, v))
                if a == addr and t == fb_type:
                    return v
            time.sleep_ms(10)
        print("   (超时无匹配反馈)")
        return None


class P:                                  # 最小参数桩, 不依赖 params.py
    d = {"f32c": {"yaw_id": 1, "pitch_id": 2, "baud": 115200,
                  "axes": 2,              # ← 3=双轴 1=仅YAW(ID1) 2=仅PITCH(ID2)
                                          #   当前=2: ID1 已烧; 新电机到货改回 3
                  "speed_rpm": 20, "max_step_deg": 3.0,
                  "yaw_lim": 60.0, "pitch_lim": 40.0,
                  "kp": 0.08, "kd": 0.02},
         "gimbal": {"inv_x": 0, "inv_y": 0, "dead_px": 3}}


print("== F32C 隔离测试 ==  引脚: %s=UART1_TX  %s=UART1_RX  %s" % (f32c.PIN_TX, f32c.PIN_RX, f32c.DEV))
g = F32CTest(P())
print("初始化完成 warn=[%s] (noFB=没收到反馈, 见文件头判读)" % g.warn)

for a, name in [x for x in ((g.id_yaw, "YAW"), (g.id_pitch, "PITCH"))
                if (x[1] == "YAW" and g.use_yaw) or (x[1] == "PITCH" and g.use_pitch)]:
    v = g._read_fb(a, FB_VOLTAGE)
    print("%s 母线电压: %s" % (name, "%.2fV" % (v / 100.0) if v is not None else "无反馈"))
    v = g._read_fb(a, FB_MULTI_ANGLE)
    print("%s 当前角度: %s" % (name, "%.1f°" % (v / 10.0) if v is not None else "无反馈"))

def sweep(addr, name, targets):
    print("-- %s 扫摆 (记录: 角度增大云台往哪边转) --" % name)
    for target in targets:
        print("%s -> %+.1f°" % (name, target))
        g._tx(addr, CMD_MULTI_POS, _i32be(round(target * 10)))
        time.sleep_ms(1500)
        v = g._read_fb(addr, FB_MULTI_ANGLE)      # 读回实际角度, 自己验自己
        if v is None:
            print("   实际角度: 无反馈")
        else:
            act = v / 10.0
            print("   实际角度: %+.1f°  误差 %+.1f°  %s" % (
                act, act - target,
                "到位 ✓" if abs(act - target) < 2.0 else "★ 没到位! 见下方判读"))
    print("   (若实际角度一直是 0.0 → 电机收到了指令但没动: 查是否使能/供电/堵转)")


if g.use_yaw:
    sweep(g.id_yaw, "YAW", (20.0, -20.0, 0.0))

if g.use_pitch:
    sweep(g.id_pitch, "PITCH", (10.0, -10.0, 0.0))

print("回零并失能")
g.center()
time.sleep_ms(1500)
g.disable()
print("== 完成 ==")
