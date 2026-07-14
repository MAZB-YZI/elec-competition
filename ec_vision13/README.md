# ec_vision — 电赛视觉框架(MaixCAM / MaixPy v4)

一套"任何控制题即插即用"的比赛级骨架:状态机多模式 + 触屏调参(JSON 持久化)+
MSPM0 串口协议 + 看门狗/相机自恢复 + 独立云台模式 + PC 主控模拟器。

## 文件

| 文件 | 作用 |
|---|---|
| `main.py` | 主程序:状态机、五个模式、触屏 UI、协议收发、WDT、自恢复 |
| `proto.py` | 协议编解码(板端) |
| `params.py` | 参数默认值 + `/root/ec_vision_params.json` 持久化 + 可调参数注册表 |
| `gimbal.py` | 独立云台:A18(PWM6)=pitch、A19(PWM7)=yaw,增量式 PD |
| `PROTOCOL.md` | **今天就发给主控队友** |
| `mspm0_ref/vision_uart.c` | 给队友的 C 参考解析器,可直接嵌入 MSPM0 工程 |
| `host_sim/pc_host_sim.py` | 电脑冒充主控(USB-TTL),不等小车先打通协议闭环 |
| `app.yaml` | MaixVision 打包安装成 app 用(开机自启前置) |

## 第一次跑通(10 分钟)

1. MaixVision 连上板子,把整个 `ec_vision` 文件夹作为项目打开,运行 `main.py`。
2. 屏幕顶栏 6 个模式按钮;底栏 `<` `>` 换参数、`-` `+` 调值、`SV` 存盘、`EXIT` 退出。
3. 桌上放张黑线纸,点 `LINE`:应看到绿线叠加,左上角显示 err/angle。
4. 插 USB-TTL 到电脑(RX→A16,TX→A17,共地),`pip install pyserial` 后
   `python host_sim/pc_host_sim.py COM5`,输入 `m 1`——板子应切模式并回心跳,
   随后每秒滚动 LINE 数据帧。**到这一步,协议闭环已经打通,后面队友照文档接即可。**

## 各模式速记

- **IDLE**:调全局参数(曝光 ExpUs=0 为自动;AWBloc=1 锁白平衡;Fps/Baud 改后需重启程序生效)。
- **LINE / BLOB**:LAB 六值 + 面积阈值全部触屏可调,调完 `SV`,断电不丢。
- **TARGET**:绿框=最大矩形(靶),红框=激光点,输出激光相对靶心偏差;
  `TgtWcm` 填靶纸真实宽度(cm)后 unit 自动变 0.1cm(线性粗换算,D6 标定日换单应性)。
- **DETECT**:默认模型 `/root/models/yolo11n.mud`;换自己的模型改 `params.py` 里
  `detect.model` 或直接编辑 `/root/ec_vision_params.json`。首次进入加载约几秒。
- **GIMBAL**:独立云台自闭环(瞄 BLOB 阈值内最大色块到画面中心)。
  舵机独立 5V 供电、与板共地;先把 Kp 从小往上加,方向反了切 InvX/InvY。
  **比赛联调后控制权归主控,此模式只用于你单人验证。**

## 开机自启(封版日再做)

1. MaixVision:项目 → 打包/安装 APP(用 `app.yaml`,id=`ec_vision`)。
2. 跑一次源码包里的 `examples/tools/set_autostart.py`,把
   `new_autostart_app_id = "ec_vision"`;或直接在板上执行
   `echo ec_vision > /maixapp/auto_start.txt && sync`。
3. 断电重启 10 连测:上电→程序自动运行→串口有心跳,10/10 才算过。
4. 同时把 `global.wdt_ms` 设为 5000、`detect.preload` 设为 1(先加载模型再喂狗)。

## D1–D2 验收清单

- [ ] 五个模式全部能进能出,触屏调参 + 存盘 + 断电重载 OK
- [ ] PC 模拟器:`m 0..5` 全部指令生效、心跳 1Hz、数据帧连续无校验错
- [ ] LINE 模式 320×240 实测 fps ≥ 50(左上角有实时值)
- [ ] 拔相机排线再插回,程序 3 秒内自恢复(自恢复逻辑演练)
- [ ] PROTOCOL.md + vision_uart.c 已发给主控队友并讲通一遍

## 预留的升级点(按排期表)

- D6 标定日:`_proc_target()` 里的线性换算替换为 cv2 `findHomography`
  四点标定(板上可 `import cv2`),输出真实物理坐标。
- 需要远程看画面时,参考源码包 `examples/vision/streaming/http_stream.py`
  加一路 HTTP 推流(注意会吃帧率,联调时再开)。
- `send_div` 可把数据帧降到主控需要的频率,减轻 MSPM0 中断压力。
