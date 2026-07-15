# -*- coding: utf-8 -*-
"""
f32c_diag.py — 相机重启/收不到反馈 分级排查(全程不使能电机, 电机不会转/不吃电流)

背景: f32c_test.py 跑到"请求反馈"那一帧时相机重启了。
     ★ 关键判据: Python 异常只会打印 traceback 并退出, 不会让整个相机重启。
       整机重启 = 电源/复位事件, 不是代码 bug。所以本脚本先把电源因素隔离出来。

用法: 改下面 STAGE 的值, 从 -1 开始一级一级往上跑, 每级跑完不重启才进下一级。

STAGE -1: 【回路自检】不接电机!! 用一根杜邦线把 A30 和 A31 直接短接(相机自己发自己收)。
          → 验证 60V 事故有没有把相机的 UART 引脚打坏。这一级过不了, 后面都白搭。
STAGE 0: 拆掉短接线, 电机仍全部断开。跑完整收发路径。 → 验证代码/read() 本身
STAGE 1: 只给【好电机 ID2】上电, 烧掉的 ID1 彻底断开(电源+信号都拔)。不使能, 只读反馈。
         → 验证接线/共地/总线/协议; 这一级能读到电压 = 硬件链路全好
STAGE 2: 同 STAGE 1, 但使能电机(会有保持力矩, 不转)。 → 验证使能瞬间的电流冲击

★★ 前提: 相机 GND 必须和电机 GND / 12V 电源 GND 连在一起。只接 RX/TX 不接 GND,
   信号没有回流路径, 电流会从相机 IO 的保护二极管灌进去 —— 既收不到数据, 又可能重启。
★★ 烧掉的 ID1 绝对不要再接相机: 它被 60V 打过, TX 脚可能在输出远超 3.3V 的电压, 会把相机也打坏。
"""
from maix import uart, pinmap, err, time
import f32c
from f32c import frame, bcc, FbParser, CMD_FEEDBACK, CMD_DISABLE
from f32c import FB_VOLTAGE, FB_MULTI_ANGLE

STAGE = -1         # ← 改这里: -1(回路自检) / 0 / 1 / 2
MOTOR_ID = 2       # ← 【好电机】的地址。ID1(X/YAW)已烧, 存活的是 ID2(Y/PITCH)


def hexs(b):
    return " ".join("%02X" % x for x in (b or b""))


print("=" * 52)
print("STAGE %d  电机地址=%d  引脚 %s=TX %s=RX %s" % (STAGE, MOTOR_ID, f32c.PIN_TX, f32c.PIN_RX, f32c.DEV))
print("=" * 52)

# --- 只开串口, 不碰电机 ---
err.check_raise(pinmap.set_pin_function(f32c.PIN_TX, "UART1_TX"), "pinmap TX")
err.check_raise(pinmap.set_pin_function(f32c.PIN_RX, "UART1_RX"), "pinmap RX")
u = uart.UART(f32c.DEV, 115200)
fb = FbParser()
print("[1] 串口已打开, 未重启 → pinmap/UART1 本身没问题")


def ask(fb_type, name, timeout_ms=600):
    f = frame(MOTOR_ID, CMD_FEEDBACK, bytes([fb_type]))
    print("  TX ->", hexs(f))
    u.write(f)
    t0 = time.ticks_ms()
    got = None
    while time.ticks_ms() - t0 < timeout_ms:
        try:
            d = u.read()
        except Exception as e:
            print("  read() 异常:", e)
            d = None
        if d:
            print("  RX(raw) <-", hexs(d))
        for a, t, v in fb.feed(d):
            print("     解析 addr=%d type=%d value=%d" % (a, t, v))
            if a == MOTOR_ID and t == fb_type:
                got = v
        if got is not None:
            break
        time.sleep_ms(10)
    print("  %s: %s" % (name, "无反馈(超时)" if got is None else got))
    return got


if STAGE == -1:
    # 回路自检: A30 和 A31 用杜邦线直接短接, 不接电机任何东西。
    # 相机自己发的字节应该原样自己收回来。收得回 = TX/RX 两个引脚都活着。
    print("[2] 回路自检: 确认 A30 和 A31 已用杜邦线短接, 且电机完全没接!")
    print("    (若没短接, 这一级必然收不到, 属正常)")
    ok_n = 0
    for i in range(5):
        probe = bytes([0x5A, 0xA5, 0x00, 0xFF, i])     # 含 0x00/0xFF 可查线是否粘连
        u.write(probe)
        time.sleep_ms(50)
        got = b""
        t0 = time.ticks_ms()
        while time.ticks_ms() - t0 < 200:
            try:
                d = u.read()
            except Exception as e:
                print("  read() 异常:", e)
                d = None
            if d:
                got += bytes(d)
            time.sleep_ms(5)
        same = got == probe
        ok_n += 1 if same else 0
        print("  第%d轮 发 %s | 收 %s  %s" % (i + 1, hexs(probe), hexs(got) or "(空)",
                                             "一致 ✓" if same else "不一致 ✗"))
    print("[3] 回环 %d/5 通过" % ok_n)
    if ok_n == 5:
        print("    ★ 相机 A30/A31 两个引脚都健康, 60V 没打坏相机。可以进 STAGE 0")
    else:
        print("    ⚠ 短接了却收不回 → 相机 UART 引脚可能已被 60V 打坏。")
        print("      换一组引脚试(改 f32c.py 里 PIN_TX/PIN_RX + DEV), 或换相机。")

elif STAGE == 0:
    # 电机断电时, 跑和 STAGE1 完全一样的收发路径(只是没有接收方)。
    # 这一级的意义: 把 u.read() 这条代码路径单独验一遍。
    #   跑完不重启 → read() 没问题, 重启因素一定在电机侧(电源/线)
    #   这一级就重启 → 与电机无关, 是板子/驱动/read() 的问题, 到时再查代码
    print("[2] 电机应处于断电状态。跑 10 轮 发送+read(), 观察是否重启...")
    for i in range(10):
        u.write(frame(MOTOR_ID, CMD_FEEDBACK, bytes([FB_VOLTAGE])))
        t0 = time.ticks_ms()
        while time.ticks_ms() - t0 < 100:
            try:
                d = u.read()          # ← 就是 f32c_test 重启时正在跑的那一行
            except Exception as e:
                print("  read() 异常:", e)
                d = None
            if d:
                print("  RX(raw) <-", hexs(d), "(电机没上电却收到数据?? 记下来)")
            time.sleep_ms(10)
        print("  第%d轮 OK" % (i + 1))
    print("[3] 收发路径跑完没重启 → 代码/read()/引脚都正常, 重启因素在电机侧")

elif STAGE >= 1:
    print("[2] 先发失能(确保电机不会突然用力), 再读反馈")
    u.write(frame(MOTOR_ID, CMD_DISABLE))
    time.sleep_ms(100)

    v = ask(FB_VOLTAGE, "母线电压")
    if v is not None:
        print("    → 折算 %.2fV" % (v / 100.0))
        print("    ★ 收到电压反馈 = 接线正确 + 共地正常 + 协议正确 + 总线双向通")
        if v < 1000:
            print("    ⚠ 低于 10V: 电源偏低或被拉垮, 重启很可能就是它")
    else:
        print("    → 无反馈。依次排查: ①黄绿两根信号线对调 ②共地 ③电机ID是否=%d" % MOTOR_ID)
        print("      ④烧掉的电机是否还挂在总线上(它可能把总线拉死, 必须整个拔掉)")

    ask(FB_MULTI_ANGLE, "当前多圈角度(0.1°)")

    if STAGE == 2:
        print("[3] 使能电机(会有保持力矩但不转)。若这一步重启 → 使能瞬间电流冲击 = 电源问题")
        u.write(frame(MOTOR_ID, 0x06))
        time.sleep_ms(500)
        print("    使能后未重启 ✓")
        ask(FB_VOLTAGE, "使能后母线电压")
        print("    ★ 对比使能前后的电压: 掉超过 1V 说明电源带不动")
        u.write(frame(MOTOR_ID, CMD_DISABLE))
        print("    已失能")

print("== STAGE %d 完成, 没有重启 ==" % STAGE)
