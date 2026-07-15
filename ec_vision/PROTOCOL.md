# 视觉 ↔ 主控(MSPM0) 串口协议 v1.1

> 这份文档是双方开发的唯一契约。任何修改必须改版本号并同步两端。
> 参考实现:板端 `proto.py`,MSPM0 端 `mspm0_ref/vision_uart.c`,PC 模拟器 `host_sim/pc_host_sim.py`。

## 1. 物理层

| 项 | 值 |
|---|---|
| 电平 | 3.3V TTL(MSPM0 原生 3.3V,直连) |
| 参数 | 115200-8N1(可在板端 IDLE 页调,两端一致即可) |
| 接线(初代 MaixCAM) | `A16`(UART0_TX) → 主控 RX;`A17`(UART0_RX) ← 主控 TX;**GND 共地** |
| 接线(**MaixCAM2**) | **左排 2×6P**:`B0`(U2T/UART2_TX) → 主控 RX;`B1`(U2R/UART2_RX) ← 主控 TX;`GND` 共地。节点 `/dev/ttyS2`。**IO 3.3V,勿接 5V**。(UART0=系统打印勿占;UART1 与云台 F32C 复用) |
| **主控(MSPM0)侧落点** | MSPM0 的 **UART0(PA0/PA1=陀螺仪 JY61P)和 UART1(PB6/PB7=调试)已占用**,视觉链路**必须**接空闲 UART:**推荐 MCU UART3 = PB2/PB3**(扩展板连接器丝印为"UART1",别被编号骗了),或 MCU UART2 = PA23/PA24(注意 PA23 兼 VREF+,启 UART2 时勿开 VREF)。 |

## 1.5 待办: 画圆相位下行(2025 E题 发挥3)

题目要求"小车行驶 1 圈,正好画 1 圈光斑,**同步误差必须小于 1/4 圈**"。
光斑的圆相位必须跟着小车的圈位置走,而**只有主控知道车跑到哪了**(它在巡线)。

现在下行只有 `0x80 SET_MODE` / `0x81 PING`,**没有传相位的通道**。视觉侧暂时用本地计时
(GIMB 页 `CircT` 秒/圈)顶着,只够静态验证 —— 车速一飘,同步误差就会累积。

**建议新增**(待与主控确认):

| 命令 | 方向 | payload | 说明 |
|---|---|---|---|
| `0x82 SET_PHASE` | 主控→视觉 | `<B phase>` | 0~255 映射 0~360°,小车圈位置。建议 20~50Hz 下发 |

视觉收到后直接用它当画圆相位,替代本地计时。

## 2. 帧格式(所有多字节字段一律小端)

```
0xAA 0x55 | LEN(1B) | CMD(1B) | PAYLOAD(LEN 字节) | SUM(1B)
SUM = (LEN + CMD + PAYLOAD各字节之和) & 0xFF
```

## 3. 视觉 → 主控(数据帧,默认每帧图像发一次;心跳固定 1Hz)

| CMD | 名称 | PAYLOAD 布局 | 说明 |
|---|---|---|---|
| 0x01 | LINE | `int16 err_x; int16 angle_x10; uint8 valid` | err_x:线中点相对画面中心横向偏差(px,右正);angle_x10:角度×10(度) |
| 0x02 | BLOB | `int16 cx,cy,w,h; uint8 valid` | 最大色块中心与外接框(px) |
| 0x03 | TARGET | `int16 dx,dy; uint8 unit; uint8 valid` | TARGET 模式: 激光点相对靶心偏差;unit=0 像素,unit=1 表示 0.1cm。**GIMBAL 模式(Src=靶心)**: dx,dy = 靶心相对瞄准点的像素偏差(unit=0),主控可据此蜂鸣/判定已对准 |
| 0x04 | DETECT | `uint8 cls; int16 cx,cy; uint8 score; uint8 valid` | 最高分目标;score 0~100 |
| 0x0F | HEARTBEAT | `uint8 mode; uint8 fps` | 1Hz;mode 见下表 |

valid=0 时其余字段一律视为无效,主控侧务必判 valid。

## 4. 主控 → 视觉(指令帧)

| CMD | 名称 | PAYLOAD | 行为 |
|---|---|---|---|
| 0x80 | SET_MODE | `uint8 mode` | 切模式,视觉立即回一帧 HEARTBEAT 作为 ACK |
| 0x81 | PING | 空 | 视觉立即回一帧 HEARTBEAT |

mode:`0 IDLE | 1 LINE | 2 BLOB | 3 TARGET | 4 DETECT | 5 GIMBAL(视觉自闭环: 云台舵机由视觉板直接 PWM 驱动, 打靶题用此模式; 主控只需收 TARGET 帧判稳)`

## 5. 帧示例(可拿逻辑分析仪逐字节对)

主控发"切到 TARGET 模式":

```
AA 55 01 80 03 84        // SUM = (01+80+03)&FF = 84
```

视觉回心跳(mode=3, fps=60):

```
AA 55 02 0F 03 3C 50     // SUM = (02+0F+03+3C)&FF = 50
```

视觉 LINE 帧,err_x=-12(0xFFF4), angle=+3.5°(35=0x0023), valid=1:

```
AA 55 05 01 F4 FF 23 00 01 1D
```

## 6. 双方约定

1. 坐标系:图像左上为原点,x 右正、y 下正;画面中心 = (w/2, h/2)。
2. 丢帧容忍:主控若 500ms 未收到任何数据帧/心跳,应进入安全策略(停车/云台保持)。
3. 校验失败静默丢弃,不回错误帧。
4. 主控解析必须是逐字节状态机(见 vision_uart.c),禁止按固定长度阻塞读。

## 7. 版本记录

- v1.1 (2026-07-14): GIMBAL 模式下(跟踪源=靶心)视觉按帧发送 TARGET 帧,dx,dy 语义为
  靶心相对瞄准点(画面中心+校靶偏移)的像素偏差,unit=0;帧结构不变,主控解析无需改动。
- v1.0: 初版。
