# -*- coding: utf-8 -*-
"""
collect_data.py — 板端采集训练数据(MaixCAM2, 独立运行)
用途: MaixHub 拍照采集不支持 MaixCAM2, 故用本脚本在板子上自采集。
      用板子摄像头采集 = 训练数据与实测同源, 避免颜色/光照偏移导致的误检。

操作(触屏):
  START/STOP  开始/暂停自动采集(每 interval 帧存一张)
  CLASS       切换当前类别名(存到对应子文件夹, 便于标注时归类)
  SHOT        手动单张拍摄
  EXIT        退出

存储: /root/collect/<类别>/<类别>_<序号>.jpg
采完把 /root/collect 整个拷到电脑, 上传 MaixHub 标注训练。

提示: 采集时慢慢转角度、变远近、换光照、多背景; 每类 50-100 张。
      易混类别(如 green/blue)让它们同框出现, 帮模型学区分。
"""
from maix import camera, display, image, touchscreen, app, time
import os

# 按你的实际类别改这里
CLASSES = ["pink", "blue", "green", "orang"]
SAVE_ROOT = "/root/collect"
INTERVAL_FRAMES = 10          # 自动采集: 每 N 帧存一张
CAM_W, CAM_H = 640, 480       # 采集用稍大分辨率, 标注更清晰


class Btn:
    def __init__(s, x, y, w, h, label):
        s.x, s.y, s.w, s.h, s.label = x, y, w, h, label
        s.dp = None
    def mapd(s, iw, ih, dw, dh):
        s.dp = image.resize_map_pos(iw, ih, dw, dh, image.Fit.FIT_CONTAIN, s.x, s.y, s.w, s.h)
    def hit(s, x, y):
        p = s.dp
        return p and p[0] < x < p[0]+p[2] and p[1] < y < p[1]+p[3]
    def draw(s, img, on=False):
        c = image.COLOR_GREEN if on else image.COLOR_WHITE
        img.draw_rect(s.x, s.y, s.w, s.h, c, 1)
        img.draw_string(s.x+3, s.y+4, s.label, c)


def main():
    cam = camera.Camera(CAM_W, CAM_H)
    disp = display.Display()
    ts = touchscreen.TouchScreen()
    iw, ih = CAM_W, CAM_H
    dw, dh = disp.width(), disp.height()

    # 计数器: 每类已存多少张(接续已有文件)
    counts = {}
    for c in CLASSES:
        d = os.path.join(SAVE_ROOT, c)
        os.makedirs(d, exist_ok=True)
        counts[c] = len([f for f in os.listdir(d) if f.endswith(".jpg")])

    cls_idx = 0
    collecting = False
    frame_i = 0
    last_msg = ""

    btns = [
        Btn(0, 0, 70, 22, "START"),
        Btn(72, 0, 70, 22, "CLASS"),
        Btn(144, 0, 60, 22, "SHOT"),
        Btn(iw-60, 0, 60, 22, "EXIT"),
    ]
    for b in btns:
        b.mapd(iw, ih, dw, dh)

    pressed = False
    press_xy = (0, 0)

    def save_one():
        nonlocal last_msg
        c = CLASSES[cls_idx]
        n = counts[c]
        path = os.path.join(SAVE_ROOT, c, "%s_%04d.jpg" % (c, n))
        try:
            img.save(path)
            counts[c] = n + 1
            last_msg = "saved %s #%d" % (c, n)
        except Exception as e:
            last_msg = "save err: " + str(e)[:20]

    while not app.need_exit():
        img = cam.read()
        frame_i += 1
        c = CLASSES[cls_idx]

        # 自动采集
        if collecting and frame_i % INTERVAL_FRAMES == 0:
            save_one()

        # 触屏
        x, y, p = ts.read()
        if p:
            pressed = True
            press_xy = (x, y)
        elif pressed:
            pressed = False
            px, py = press_xy
            for b in btns:
                if b.hit(px, py):
                    if b.label == "START":
                        collecting = not collecting
                        b.label = "STOP" if collecting else "START"
                    elif b.label in ("STOP",):
                        collecting = not collecting
                        b.label = "STOP" if collecting else "START"
                    elif b.label == "CLASS":
                        cls_idx = (cls_idx + 1) % len(CLASSES)
                    elif b.label == "SHOT":
                        save_one()
                    elif b.label == "EXIT":
                        app.set_exit_flag(True)
                    break

        # UI
        for b in btns:
            on = (b.label == "STOP")
            b.draw(img, on)
        img.draw_string(2, 26, "class:%s  total:%d  %s" % (c, counts[c], last_msg),
                        image.COLOR_YELLOW)
        img.draw_string(2, 42, "collecting..." if collecting else "paused (START to go)",
                        image.COLOR_GREEN if collecting else image.COLOR_WHITE)
        disp.show(img, fit=image.Fit.FIT_CONTAIN)

    print("collected:", counts)


if __name__ == "__main__":
    main()
