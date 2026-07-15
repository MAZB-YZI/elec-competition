python extract_frames.py 你的视频.mp4

# 每 0.3 秒抽一张,最多 100 张,存到指定文件夹
python extract_frames.py video.mp4 -s 0.3 -m 100 -o dataset/red_block

# 多个视频一起处理(不同角度分开录的)
python extract_frames.py v1.mp4 v2.mp4 v3.mp4 -o dataset/cup



 "C:\Users\24017\AppData\Local\Programs\Python\Python312\python.exe" extract_frames.py test.mp4 -s 0.3 -m 100 -o dataset/test 