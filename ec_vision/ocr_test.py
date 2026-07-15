# -*- coding: utf-8 -*-
"""ocr_test.py — PaddleOCR 数字/字符识别 隔离测试

严格照抄官方例程(wiki.sipeed.com/maixpy/doc/zh/vision/ocr.html), 先验通再谈集成。
为什么不直接改框架: 官方例程里相机是按 ocr.input_width()/input_height() 建的,
而框架的相机已按别的分辨率建好了 —— 这两者能不能共存我没验过, 不猜。

用法: python ocr_test.py     按 Ctrl+C 退出
     拿一张写着数字的纸对着相机, 看终端打印。

模型三选一(改下面 MODEL):
  pp_ocr_en.mud      英文+数字, 最小最快 ★数字识别用这个
  pp_ocr.mud         中文+英文+数字, 大一些
  pp_ocr_320x224.mud 320x224 输入, 分辨率低但更快
"""
from maix import camera, display, image, nn, app

MODEL = "/root/models/pp_ocr_en.mud"

ocr = nn.PP_OCR(MODEL)
print("model:", MODEL)
print("模型输入尺寸: %dx%d  format=%s" % (ocr.input_width(), ocr.input_height(), ocr.input_format()))
print("★ 记下这个尺寸 —— 集成进框架时相机分辨率要和它对得上\n")

cam = camera.Camera(ocr.input_width(), ocr.input_height(), ocr.input_format())
disp = display.Display()

def digits_only(s):
    """只保留 0-9。实测发现: '1 2 3' 识别完美, 但单个 '8' 常被读成 '8D'/'8N'/'8M' ——
    尾巴上总粘一个假字母(det 框略大 或 rec 幻觉出第二个字符)。
    病房号/编号这类场景只要数字, 直接把非数字滤掉即可, 一行解决。"""
    return "".join(c for c in s if c.isdigit())

last = ""
while not app.need_exit():
    img = cam.read()
    objs = ocr.detect(img)
    raw = " | ".join(o.char_str() for o in objs)
    num = " ".join(d for d in (digits_only(o.char_str()) for o in objs) if d)
    if num and num != last:            # 只在结果变化时打印, 免得刷屏
        print("数字: %-12s   (原始: %s)" % (num, raw[:50]))
        last = num
    for o in objs:
        img.draw_keypoints(o.box.to_list(), image.COLOR_RED, 4, -1, 1)
        img.draw_string(o.box.x4, o.box.y4, o.char_str(), image.COLOR_RED)
    disp.show(img)
