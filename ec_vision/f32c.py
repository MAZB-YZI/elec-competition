# -*- coding: utf-8 -*-
"""
f32c.py — WHEELTEC F32C TTL 无刷云台电机: MaixCAM2 直连驱动 + 视觉外环
协议来源: 已用厂商《F32C无刷电机/云台使用手册 V1.0》+ 官方例程 MiniBalance.c 逐帧核对,
          与 MSPM0 参考实现三方一致(每条指令字节级对拍通过, 见 f32c_verify 测试):
    发送帧: 0x7A | 地址 | 命令 | 数据(大端) | BCC | 0x7B
    BCC   = 从 0x7A 到数据末字节 全部异或
    反馈帧: 固定 9 字节: 0x7A | 地址 | 类型 | 数据4B(大端) | BCC(前7字节) | 0x7B
    角度单位: 0.1°/LSB (int32, 多圈可无限旋转 —— 因此必须软件限位保护相机排线!)
    两电机同总线, 地址区分: YAW=0x01, PITCH=0x02 (厂商软件预设)
    串口: 115200-8N1 (与 MSPM0 参考工程 syscfg 一致)
接线(MaixCAM2):
    右排 A30/A31 复用 UART1 —— 用 F32C 时不能再用 PWM6/7(与 PWM 舵机方案互斥)。
    默认映射 A30=UART1_TX -> 电机RX, A31=UART1_RX <- 电机TX, 节点 /dev/ttyS1。
    ★ 该映射未经实机验证: 若 f32c_test.py 收不到反馈, 先对调两根信号线再试。
    电机 12V 独立供电, 与 MaixCAM2 共地。12V 绝不可接到 MaixCAM2 任何引脚!
安全设计:
    - 上电时序: 失能 -> 多圈角度清零 -> 使能 -> 位置模式 -> 限速 -> 目标=0(原地保持)
      清零在失能状态下做, 避免位置环带着旧目标跳变。
    - 反馈可选: 视觉闭环不依赖电机反馈(纯 TX 也能跑), 反馈仅用于零点校验。
    - yaw_lim / pitch_lim 软件限位: 多圈电机会无限转, 不限位会绞断相机排线。
手册确认要点:
    - 位置模式默认"多圈直通"(mode 3): 手册明确"高频改目标位置建议直通", 正是视觉伺服场景;
      官方例程用的是 mode 1(T型), 若直通发抽可切回。改 pos_mode 后重启生效。
    - 帧间隔 INTER_FRAME_MS: 手册要求每帧留间隔(DMA空闲中断), 已在连发处补足。
    - 位置指令/多圈反馈 0.1°/LSB(×10), 电压反馈 0.01V/LSB(×100), 已对齐手册示例帧。
    - 12V 下最大可控转速 1000RPM, speed_rpm 默认 30 远在安全区。
"""
from maix import uart, pinmap, err, time

HEAD, TAIL = 0x7A, 0x7B
CMD_MODE, CMD_SPEED, CMD_MULTI_POS, CMD_SINGLE_POS = 0x00, 0x01, 0x02, 0x03
CMD_DISABLE, CMD_ENABLE, CMD_ACC, CMD_SAVE = 0x05, 0x06, 0x07, 0x08
CMD_CLEAR_MULTI, CMD_SET_ZERO, CMD_FACTORY_RST, CMD_SET_ADDR = 0x09, 0x0A, 0x0B, 0x0D
CMD_FEEDBACK = 0x0E
MODE_SPEED, MODE_MULTI_POS, MODE_MULTI_POS_L = 0, 1, 3   # 1=多圈T型规划 3=多圈直通(高频改目标推荐)
FB_SPEED, FB_MULTI_ANGLE, FB_SINGLE_ANGLE, FB_ACC, FB_VOLTAGE = 0, 1, 2, 3, 4

# 引脚映射待实机确认(丝印为准): 收不到反馈先对调电机侧两根信号线
PIN_TX, PIN_RX, DEV = "A30", "A31", "/dev/ttyS1"

# 帧间隔: 电机 RX 走 DMA 空闲中断, 连发两帧不留间隔会被合并/丢弃。
# 厂商手册"多电机级联"要求每帧间隔 >=1ms; 官方例程 MiniBalance.c 用了 10ms。
# 取 3ms: 远高于 1ms 下限, 又不吃满视觉 40ms 帧预算。
INTER_FRAME_MS = 3


def bcc(data):
    v = 0
    for b in data:
        v ^= b
    return v & 0xFF


def frame(addr, cmd, data=b""):
    body = bytes([HEAD, addr & 0xFF, cmd & 0xFF]) + bytes(data)
    return body + bytes([bcc(body), TAIL])


def _i32be(v):
    v = int(v) & 0xFFFFFFFF
    return bytes([(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def _i16be(v):
    v = int(v) & 0xFFFF
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


class FbParser:
    """反馈帧解析: 固定9字节 7A|addr|type|d4|BCC|7B, 与 MSPM0 uart_callback.c 同逻辑。"""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        out = []
        if data:
            self.buf += data
        while len(self.buf) >= 9:
            if self.buf[0] != HEAD:
                self.buf.pop(0)
                continue
            f = self.buf[:9]
            if f[8] == TAIL and bcc(f[:7]) == f[7]:
                raw = (f[3] << 24) | (f[4] << 16) | (f[5] << 8) | f[6]
                if raw >= 0x80000000:
                    raw -= 0x100000000          # int32
                out.append((f[1], f[2], raw))    # (addr, type, value)
                del self.buf[:9]
            else:
                self.buf.pop(0)                  # 滑动重同步
        return out


class F32CGimbal:
    """与 gimbal.Gimbal 同接口(center/update/hold/status_str), 供 main.py 无差别调用。"""

    def __init__(self, p):
        self.p = p
        f = p.d["f32c"]
        self.id_yaw = int(f.get("yaw_id", 1))
        self.id_pitch = int(f.get("pitch_id", 2))
        # axes: 哪些轴实际在线。3=双轴(默认) 1=仅YAW 2=仅PITCH。
        # 电机烧坏/拆掉时必须设对: 驱动不会向不在线的地址发任何帧, 也不等它的反馈。
        axes = int(f.get("axes", 3))
        self.use_yaw = bool(axes & 1)
        self.use_pitch = bool(axes & 2)
        # UART1 引脚映射; 若 pinmap 抛错说明引脚名/功能名不对, 报到上层显示
        err.check_raise(pinmap.set_pin_function(PIN_TX, "UART1_TX"), "pinmap %s UART1_TX" % PIN_TX)
        err.check_raise(pinmap.set_pin_function(PIN_RX, "UART1_RX"), "pinmap %s UART1_RX" % PIN_RX)
        self.u = uart.UART(DEV, int(f.get("baud", 115200)))
        self.fb = FbParser()
        self.ang_yaw = 0.0          # 当前指令角(度, 相对上电清零点)
        self.ang_pitch = 0.0
        self.last_ex = 0.0
        self.last_ey = 0.0
        self.limited = False        # 上帧是否顶到软件限位
        self.warn = ""              # "" 或 "noFB": 反馈校验失败(纯TX模式, 仍可用)
        self._init_motors()

    def _live(self):
        """在线电机地址列表(顺序: yaw, pitch)。"""
        a = []
        if self.use_yaw:
            a.append(self.id_yaw)
        if self.use_pitch:
            a.append(self.id_pitch)
        return a

    # ---- 底层 ----
    def _tx(self, addr, cmd, data=b"", gap_ms=0):
        self.u.write(frame(addr, cmd, data))
        if gap_ms:
            time.sleep_ms(gap_ms)

    def _both(self, cmd, data=b"", gap_ms=20):
        """只发给在线电机(单轴时不会去戳烧掉/拆掉的地址)。"""
        for a in self._live():
            self._tx(a, cmd, data, gap_ms)

    def _read_fb(self, addr, fb_type, timeout_ms=300):
        """请求并等待一条反馈; 超时返回 None。"""
        # ★ 必须先留帧间隔: 调用方常常刚发完位置帧就来读反馈, 两帧贴在一起会被电机的
        #   DMA 空闲中断当成一坨收, 后一帧解析不出来 => 明明线是通的却读不到反馈。
        time.sleep_ms(INTER_FRAME_MS)
        self._tx(addr, CMD_FEEDBACK, bytes([fb_type]))
        t0 = time.ticks_ms()
        while time.ticks_ms() - t0 < timeout_ms:
            try:
                data = self.u.read()
            except Exception:
                data = None
            for a, t, v in self.fb.feed(data):
                if a == addr and t == fb_type:
                    return v
            time.sleep_ms(10)
        return None

    # ---- 上电时序(移植自 MSPM0 empty.c, 清零改在失能态做更安全) ----
    def _init_motors(self):
        self.u.write(b"\x00")                       # 同步字节(参考实现同款)
        time.sleep_ms(20)
        self._both(CMD_DISABLE)                     # 1. 失能
        self._both(CMD_CLEAR_MULTI)                 # 2. 多圈角度清零(当前姿态=0点)
        self._both(CMD_ENABLE)                      # 3. 使能
        # 4. 位置模式: pos_mode=3(多圈直通)为默认——手册明确"高频率改目标位置建议直通",
        #    正是视觉伺服场景;外环已用 max_step_deg 自己限速, 不需要电机内部T型规划再叠加延迟。
        #    pos_mode=1(多圈T型)是官方例程用的模式, 若直通感觉发抽可切回1。改后重启生效。
        mode = int(self.p.d["f32c"].get("pos_mode", MODE_MULTI_POS_L))
        self._both(CMD_MODE, _i16be(mode))
        spd = int(self.p.d["f32c"].get("speed_rpm", 30))
        self._both(CMD_SPEED, _i16be(spd))          # 5. 位置模式限速
        self._send_pos()                            # 6. 目标=0, 原地保持
        # 7. 反馈校验清零是否生效(|角度|应≈0); 失败仅告警, 不阻塞(纯TX可用)
        ok = True
        for a in self._live():
            v = self._read_fb(a, FB_MULTI_ANGLE)
            if v is None or abs(v) > 100:           # 0.1°单位, >10°视为清零未生效
                ok = False
        self.warn = "" if ok else "noFB"

    def _send_pos(self):
        # 两帧之间必须留间隔, 否则电机 DMA 空闲中断收不全(见 INTER_FRAME_MS 说明)
        if self.use_yaw:
            self._tx(self.id_yaw, CMD_MULTI_POS, _i32be(round(self.ang_yaw * 10)),
                     gap_ms=INTER_FRAME_MS if self.use_pitch else 0)
        if self.use_pitch:
            self._tx(self.id_pitch, CMD_MULTI_POS, _i32be(round(self.ang_pitch * 10)))

    # ---- Gimbal 同接口 ----
    def center(self):
        self.ang_yaw = 0.0
        self.ang_pitch = 0.0
        self.last_ex = 0.0
        self.last_ey = 0.0
        self.limited = False
        self._send_pos()

    def update(self, err_x, err_y):
        """err_x/err_y: 目标相对瞄准点像素误差(右/下为正)。每帧调用一次。"""
        g = self.p.d["gimbal"]
        f = self.p.d["f32c"]
        dead = float(g.get("dead_px", 3))
        if abs(err_x) <= dead:
            err_x = 0.0
        if abs(err_y) <= dead:
            err_y = 0.0
        sx = -1.0 if g["inv_x"] else 1.0
        sy = -1.0 if g["inv_y"] else 1.0
        dx = f["kp"] * err_x + f["kd"] * (err_x - self.last_ex)     # 度/帧
        dy = f["kp"] * err_y + f["kd"] * (err_y - self.last_ey)
        self.last_ex = err_x
        self.last_ey = err_y
        ms = float(f.get("max_step_deg", 3.0))      # 单帧角度步进上限
        dx = max(-ms, min(ms, dx))
        dy = max(-ms, min(ms, dy))
        yl = float(f.get("yaw_lim", 60.0))          # 软件限位: 保护相机排线
        pl = float(f.get("pitch_lim", 40.0))
        # 只积分【在线】的轴。离线轴若继续累加, 其指令角会一路飘到限位, 既没意义又污染日志
        # (Axes=2 时 ang_yaw 曾飘到 ±60, 让人误以为 YAW 在动)。
        if self.use_yaw:
            self.ang_yaw = max(-yl, min(yl, self.ang_yaw + sx * dx))
        if self.use_pitch:
            self.ang_pitch = max(-pl, min(pl, self.ang_pitch + sy * dy))
        # 只统计在线轴的限位, 否则离线轴会误报 LIM
        self.limited = ((self.use_yaw and abs(self.ang_yaw) >= yl - 1e-6) or
                        (self.use_pitch and abs(self.ang_pitch) >= pl - 1e-6))
        self._send_pos()

    def hold(self):
        """丢目标: 保持当前位置, 清微分防重捕获踢一脚。"""
        self.last_ex = 0.0
        self.last_ey = 0.0

    def status_str(self):
        s = ""
        if not (self.use_yaw and self.use_pitch):   # 单轴运行中, 提醒另一轴不受控
            s += " %s-only" % ("YAW" if self.use_yaw else "PIT")
        if self.warn:
            s += " " + self.warn
        if self.limited:
            s += " LIM"      # 顶到限位: 大概率 InvX/InvY 方向反了, 或限位设太小
        return s

    def disable(self):
        self._both(CMD_DISABLE)
