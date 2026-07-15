# -*- coding: utf-8 -*-
"""
pc_host_sim.py — 在电脑上模拟 MSPM0 主控, 与 MaixCAM 走同一套协议
用途: 队友的车还没好时, 用 USB-TTL 模块把电脑当主控, 提前把协议闭环全部打通。
接线: USB-TTL 的 RX->板A16(UART0_TX), TX->板A17(UART0_RX), GND 共地。3.3V 电平!
用法:
    pip install pyserial
    python pc_host_sim.py COM5            # Windows
    python pc_host_sim.py /dev/ttyUSB0    # Linux/Mac
    可选: python pc_host_sim.py COM5 115200
交互命令:
    m 0..5   切模式 (0 IDLE 1 LINE 2 BLOB 3 TARGET 4 DETECT 5 GIMBAL)
    ping     请求一帧心跳
    q        退出
"""
import struct
import sys
import threading
import time

import serial

HEAD0, HEAD1 = 0xAA, 0x55
CMD_LINE, CMD_BLOB, CMD_TARGET, CMD_DETECT, CMD_HEARTBEAT = 0x01, 0x02, 0x03, 0x04, 0x0F
CMD_CIRCLE, CMD_TAG = 0x05, 0x06
CMD_SET_MODE, CMD_PING = 0x80, 0x81
MODE_NAMES = ["IDLE", "LINE", "BLOB", "TARG", "DET", "GIMB", "CIRC", "TAG"]


def pack(cmd, payload=b""):
    s = (len(payload) + cmd + sum(payload)) & 0xFF
    return bytes([HEAD0, HEAD1, len(payload), cmd]) + payload + bytes([s])


class Parser:
    def __init__(self):
        self.st, self.ln, self.cmd, self.buf = 0, 0, 0, bytearray()

    def feed(self, data):
        out = []
        for b in data:
            if self.st == 0:
                self.st = 1 if b == HEAD0 else 0
            elif self.st == 1:
                self.st = 2 if b == HEAD1 else (1 if b == HEAD0 else 0)
            elif self.st == 2:
                if b > 64:
                    self.st = 0
                else:
                    self.ln, self.st = b, 3
            elif self.st == 3:
                self.cmd, self.buf = b, bytearray()
                self.st = 4 if self.ln else 5
            elif self.st == 4:
                self.buf.append(b)
                if len(self.buf) >= self.ln:
                    self.st = 5
            elif self.st == 5:
                if (self.ln + self.cmd + sum(self.buf)) & 0xFF == b:
                    out.append((self.cmd, bytes(self.buf)))
                self.st = 0
        return out


def describe(cmd, pl):
    try:
        if cmd == CMD_LINE:
            e, a, v = struct.unpack("<hhB", pl)
            return "LINE   err_x=%-5d angle=%-6.1f valid=%d" % (e, a / 10.0, v)
        if cmd == CMD_BLOB:
            cx, cy, w, h, v = struct.unpack("<hhhhB", pl)
            return "BLOB   c=(%d,%d) size=(%d,%d) valid=%d" % (cx, cy, w, h, v)
        if cmd == CMD_TARGET:
            dx, dy, u, v = struct.unpack("<hhBB", pl)
            unit = "0.1cm" if u == 1 else "px"
            return "TARGET d=(%d,%d)%s valid=%d" % (dx, dy, unit, v)
        if cmd == CMD_DETECT:
            c, cx, cy, s, v = struct.unpack("<BhhBB", pl)
            return "DETECT cls=%d c=(%d,%d) score=%d%% valid=%d" % (c, cx, cy, s, v)
        if cmd == CMD_CIRCLE:
            cx, cy, r, v = struct.unpack("<hhhB", pl)
            return "CIRCLE c=(%d,%d) r=%d valid=%d" % (cx, cy, r, v)
        if cmd == CMD_TAG:
            tid, cx, cy, v = struct.unpack("<hhhB", pl)
            return "TAG    id=%d c=(%d,%d) valid=%d" % (tid, cx, cy, v)
        if cmd == CMD_HEARTBEAT:
            m, f = struct.unpack("<BB", pl)
            return "HEART  mode=%s fps=%d" % (MODE_NAMES[m] if m < 6 else m, f)
    except Exception as e:
        return "cmd=0x%02X decode err %s" % (cmd, e)
    return "cmd=0x%02X payload=%s" % (cmd, pl.hex())


def rx_loop(ser):
    p = Parser()
    stat = {}
    last_print = {}
    while True:
        data = ser.read(256)
        if not data:
            continue
        for cmd, pl in p.feed(data):
            stat[cmd] = stat.get(cmd, 0) + 1
            now = time.time()
            # 数据帧限频打印(5Hz), 心跳全打
            if cmd == CMD_HEARTBEAT or now - last_print.get(cmd, 0) > 0.2:
                last_print[cmd] = now
                print("[RX #%-6d] %s" % (stat[cmd], describe(cmd, pl)))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    ser = serial.Serial(port, baud, timeout=0.05)
    print("opened %s @ %d — 输入 'm 3' 切靶纸模式, 'ping' 要心跳, 'q' 退出" % (port, baud))
    threading.Thread(target=rx_loop, args=(ser,), daemon=True).start()
    while True:
        try:
            line = input().strip().lower()
        except EOFError:
            break
        if line == "q":
            break
        if line == "ping":
            ser.write(pack(CMD_PING))
            print("[TX] PING")
        elif line.startswith("m "):
            try:
                m = int(line.split()[1])
                ser.write(pack(CMD_SET_MODE, bytes([m])))
                print("[TX] SET_MODE ->", MODE_NAMES[m] if m < 6 else m)
            except Exception as e:
                print("bad cmd:", e)
        elif line:
            print("未知命令。m 0..5 | ping | q")


if __name__ == "__main__":
    main()
