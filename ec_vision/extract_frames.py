# -*- coding: utf-8 -*-
"""
extract_frames.py — 视频抽帧, 生成 AI 训练数据集(电脑上运行)
依赖: pip install opencv-python

典型用法:
  # 每 15 帧抽一张(约每 0.5 秒一张, 视频 30fps 时)
  python extract_frames.py my_video.mp4

  # 抽到指定文件夹, 每 10 帧一张, 最多 100 张
  python extract_frames.py my_video.mp4 -o dataset/red_block -e 10 -m 100

  # 按时间间隔抽(每 0.3 秒一张), 并缩放到 640 宽
  python extract_frames.py my_video.mp4 -s 0.3 -w 640

  # 一次处理多个视频(不同角度分别录的)
  python extract_frames.py v1.mp4 v2.mp4 v3.mp4 -o dataset/cup

参数:
  -o/--out     输出目录(默认: frames_<视频名>)
  -e/--every   每 N 帧抽一张(和 -s 二选一)
  -s/--sec     每多少秒抽一张(和 -e 二选一, 优先)
  -m/--max     最多抽多少张(0=不限)
  -w/--width   输出图宽度(等比缩放, 0=原尺寸)。训练建议 640
  --start      从第几秒开始
  --end        到第几秒结束(0=到结尾)
  --prefix     文件名前缀(默认用视频名)
"""
import argparse
import os
import sys

try:
    import cv2
except ImportError:
    print("需要 opencv: pip install opencv-python")
    sys.exit(1)


def extract(video_path, out_dir, every, sec, max_n, width, start, end, prefix, start_index):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("  打不开视频:", video_path)
        return start_index

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # 按秒间隔优先; 否则按帧间隔
    step = max(1, int(round(sec * fps))) if sec > 0 else max(1, every)
    start_frame = int(start * fps)
    end_frame = int(end * fps) if end > 0 else total

    os.makedirs(out_dir, exist_ok=True)
    name = prefix or os.path.splitext(os.path.basename(video_path))[0]

    print("  视频 %s: %.1ffps, 共%d帧, 每%d帧抽一张" % (
        os.path.basename(video_path), fps, total, step))

    saved = start_index
    frame_i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_i < start_frame:
            frame_i += 1
            continue
        if frame_i > end_frame:
            break
        if (frame_i - start_frame) % step == 0:
            if width > 0:
                h, w = frame.shape[:2]
                nh = int(h * width / w)
                frame = cv2.resize(frame, (width, nh), interpolation=cv2.INTER_AREA)
            fn = os.path.join(out_dir, "%s_%04d.jpg" % (name, saved))
            cv2.imwrite(fn, frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            saved += 1
            if max_n > 0 and saved - start_index >= max_n:
                print("  达到上限 %d 张, 停止" % max_n)
                break
        frame_i += 1

    cap.release()
    print("  -> 本视频抽出 %d 张" % (saved - start_index))
    return saved


def main():
    ap = argparse.ArgumentParser(description="视频抽帧生成训练数据")
    ap.add_argument("videos", nargs="+", help="一个或多个视频文件")
    ap.add_argument("-o", "--out", default=None, help="输出目录")
    ap.add_argument("-e", "--every", type=int, default=15, help="每N帧抽一张")
    ap.add_argument("-s", "--sec", type=float, default=0, help="每多少秒抽一张(优先)")
    ap.add_argument("-m", "--max", type=int, default=0, help="最多抽多少张(0=不限)")
    ap.add_argument("-w", "--width", type=int, default=640, help="输出宽度(0=原尺寸)")
    ap.add_argument("--start", type=float, default=0, help="起始秒")
    ap.add_argument("--end", type=float, default=0, help="结束秒(0=到结尾)")
    ap.add_argument("--prefix", default=None, help="文件名前缀")
    args = ap.parse_args()

    out = args.out or ("frames_" + os.path.splitext(os.path.basename(args.videos[0]))[0])
    print("输出目录:", out)
    idx = 0
    for v in args.videos:
        if not os.path.exists(v):
            print("  跳过不存在的文件:", v)
            continue
        idx = extract(v, out, args.every, args.sec, args.max,
                      args.width, args.start, args.end, args.prefix, idx)
    print("\n完成! 共 %d 张 -> %s" % (idx, out))
    print("下一步: 上传到 MaixHub 标注 (见 DET_TRAINING_GUIDE.md)")


if __name__ == "__main__":
    main()
