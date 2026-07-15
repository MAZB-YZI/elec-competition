# -*- coding: utf-8 -*-
"""
diag.py — MaixCAM2 视觉诊断脚本(独立运行, 不依赖框架)
目的: 用事实而非猜测, 逐个确认:
  1) 相机分辨率/格式/帧率, 图像对象的真实属性
  2) 曝光当前值, 以及不同曝光对"重影"的影响
  3) 每个 find 函数在带 roi / 不带 roi 时是否崩溃, 逐个隔离
运行: MaixVision 里单独运行本文件, 看终端打印。每项测试独立 try 包裹,
      某项崩了(段错误除外)不影响后面。段错误会指出崩在哪一行的上一句。
"""
from maix import camera, display, image, time, sys

def line(t=""): print("=" * 8, t)

print("\n\n")
line("DIAG START  device=" + str(sys.device_id()))

# ---------- 1. 相机基本属性 ----------
line("1. camera open 320x240")
cam = camera.Camera(320, 240)
disp = display.Display()
img = cam.read()
print("  img =", img)
print("  width/height =", img.width(), img.height())
try:
    print("  format =", img.format())
except Exception as e:
    print("  format() err:", repr(e))

# ---------- 2. 曝光 ----------
line("2. exposure")
try:
    print("  current exposure =", cam.exposure())
except Exception as e:
    print("  exposure() read err:", repr(e))
# 试几档曝光, 观察画面(晃动板子看重影): 越大越亮但拖影越重
for us in [500, 1000, 2000]:
    try:
        cam.exposure(us)
        time.sleep_ms(300)
        _ = cam.read()
        print("  set exposure", us, "us -> ok (晃动板子观察此档重影程度)")
        time.sleep_ms(1200)
    except Exception as e:
        print("  set exposure", us, "err:", repr(e))
# 恢复自动
try:
    cam.exp_mode(camera.AeMode.Auto)
    print("  restored auto exposure")
except Exception as e:
    print("  restore auto err:", repr(e))

# ---------- 3. 逐个测 find 函数 (关键: 隔离段错误) ----------
# 每个测试前后各打印一行, 若段错误, 最后一条 "BEFORE xxx" 就是崩溃点
img = cam.read()

def test(name, fn):
    print("  BEFORE", name, flush=True)
    try:
        r = fn(img)
        print("  AFTER ", name, "-> ok, n =", (len(r) if r is not None else "None"))
    except Exception as e:
        print("  AFTER ", name, "-> EXC:", repr(e))

line("3. find_blobs (无 roi)")
test("find_blobs no-roi", lambda im: im.find_blobs([[0,80,40,80,10,80]], area_threshold=100, pixels_threshold=100))

line("4. find_blobs (带 roi 关键字)")
test("find_blobs roi=", lambda im: im.find_blobs([[0,80,40,80,10,80]], area_threshold=100, pixels_threshold=100, roi=[0,24,320,192]))

line("5. get_regression (无 roi)")
test("get_regression no-roi", lambda im: im.get_regression([[0,40,-128,127,-128,127]], area_threshold=100))

line("6. get_regression (带 roi 关键字)")
test("get_regression roi=", lambda im: im.get_regression([[0,40,-128,127,-128,127]], area_threshold=100, roi=[0,24,320,192]))

line("7. find_circles (无 roi)")
test("find_circles no-roi", lambda im: im.find_circles(threshold=2000))

line("8. find_rects (无 roi)  <-- 已知嫌疑, 放最后")
test("find_rects no-roi", lambda im: im.find_rects(threshold=10000))

line("9. find_rects (带 roi 位置参数)")
test("find_rects pos-roi", lambda im: im.find_rects([0,24,320,192], 10000))

line("DIAG DONE — 若在某个 BEFORE 后直接 Segmentation fault, 那句就是元凶")
