# -*- coding: utf-8 -*-
"""
proto.py — 电赛视觉<->主控(MSPM0) 串口协议编解码
帧格式(小端):
  0xAA 0x55 | LEN(1B,payload长度) | CMD(1B) | PAYLOAD(LEN B) | SUM(1B)
  SUM = (LEN + CMD + sum(PAYLOAD)) & 0xFF
详细字段定义见 PROTOCOL.md
"""
import struct

HEAD0 = 0xAA
HEAD1 = 0x55

# 视觉 -> 主控
CMD_LINE      = 0x01   # <h err_x><h angle_x10><B valid>
CMD_BLOB      = 0x02   # <h cx><h cy><h w><h h><B valid>
CMD_TARGET    = 0x03   # <h dx><h dy><B unit(0=px,1=0.1cm)><B valid>
CMD_DETECT    = 0x04   # <B class><h cx><h cy><B score0_100><B valid>
CMD_CIRCLE    = 0x05   # <h cx><h cy><h r><B valid>
CMD_TAG       = 0x06   # <h id><h cx><h cy><B valid>
CMD_HEARTBEAT = 0x0F   # <B mode><B fps>

# 主控 -> 视觉
CMD_SET_MODE  = 0x80   # <B mode>  0=IDLE 1=LINE 2=BLOB 3=TARGET 4=DETECT 5=GIMBAL
CMD_PING      = 0x81   # 空payload, 视觉立刻回一帧心跳


def pack(cmd: int, payload: bytes = b"") -> bytes:
    ln = len(payload)
    s = (ln + cmd + sum(payload)) & 0xFF
    return bytes([HEAD0, HEAD1, ln, cmd]) + payload + bytes([s])


def pack_line(err_x: int, angle_deg: float, valid: bool) -> bytes:
    return pack(CMD_LINE, struct.pack("<hhB", int(err_x), int(angle_deg * 10), 1 if valid else 0))


def pack_blob(cx: int, cy: int, w: int, h: int, valid: bool) -> bytes:
    return pack(CMD_BLOB, struct.pack("<hhhhB", int(cx), int(cy), int(w), int(h), 1 if valid else 0))


def pack_target(dx: int, dy: int, unit: int, valid: bool) -> bytes:
    return pack(CMD_TARGET, struct.pack("<hhBB", int(dx), int(dy), unit, 1 if valid else 0))


def pack_detect(cls: int, cx: int, cy: int, score01: float, valid: bool) -> bytes:
    return pack(CMD_DETECT, struct.pack("<BhhBB", cls & 0xFF, int(cx), int(cy),
                                        int(max(0.0, min(1.0, score01)) * 100), 1 if valid else 0))


def pack_circle(cx: int, cy: int, r: int, valid: bool) -> bytes:
    return pack(CMD_CIRCLE, struct.pack("<hhhB", int(cx), int(cy), int(r), 1 if valid else 0))


def pack_tag(tag_id: int, cx: int, cy: int, valid: bool) -> bytes:
    return pack(CMD_TAG, struct.pack("<hhhB", int(tag_id), int(cx), int(cy), 1 if valid else 0))


def pack_heartbeat(mode: int, fps: int) -> bytes:
    return pack(CMD_HEARTBEAT, struct.pack("<BB", mode & 0xFF, min(255, max(0, int(fps)))))


class Parser:
    """字节流状态机解析器。feed() 返回 [(cmd, payload), ...]"""
    S_H0, S_H1, S_LEN, S_CMD, S_PAYLOAD, S_SUM = range(6)

    def __init__(self, max_payload=64):
        self.max_payload = max_payload
        self._reset()

    def _reset(self):
        self.state = self.S_H0
        self.ln = 0
        self.cmd = 0
        self.buf = bytearray()

    def feed(self, data: bytes):
        out = []
        for b in data:
            if self.state == self.S_H0:
                if b == HEAD0:
                    self.state = self.S_H1
            elif self.state == self.S_H1:
                self.state = self.S_LEN if b == HEAD1 else (self.S_H1 if b == HEAD0 else self.S_H0)
            elif self.state == self.S_LEN:
                if b > self.max_payload:
                    self._reset()
                else:
                    self.ln = b
                    self.state = self.S_CMD
            elif self.state == self.S_CMD:
                self.cmd = b
                self.buf = bytearray()
                self.state = self.S_PAYLOAD if self.ln > 0 else self.S_SUM
            elif self.state == self.S_PAYLOAD:
                self.buf.append(b)
                if len(self.buf) >= self.ln:
                    self.state = self.S_SUM
            elif self.state == self.S_SUM:
                calc = (self.ln + self.cmd + sum(self.buf)) & 0xFF
                if calc == b:
                    out.append((self.cmd, bytes(self.buf)))
                self._reset()
        return out
