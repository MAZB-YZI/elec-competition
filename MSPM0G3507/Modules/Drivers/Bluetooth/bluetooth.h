/**
 * bluetooth.h — HC-05 蓝牙串口通用模块
 *
 * 提供 UART RX/TX 基础设施，命令解析由项目自定义
 * 通过宏定义 UART 实例名，适配不同项目的 SysConfig
 */

#ifndef MODULES_DRIVERS_BLUETOOTH_H
#define MODULES_DRIVERS_BLUETOOTH_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 默认配置（项目可在包含前 #define 覆盖） */
#ifndef BT_RX_BUF_SIZE
#define BT_RX_BUF_SIZE  128
#endif

#ifndef BT_TIMEOUT_MS
#define BT_TIMEOUT_MS   200     /* 无换行时的超时时间 */
#endif

/* 命令回调类型：收到完整命令时调用，返回 true 表示参数已更新 */
typedef bool (*BT_CmdHandler)(const char *cmd, void *ctx);

/* 初始化蓝牙模块 */
void BT_Init(void);

/* 设置系统 tick 指针（用于超时判断） */
void BT_SetTickPtr(volatile uint32_t *tick_ptr);

/* 设置命令处理回调 */
void BT_SetCmdHandler(BT_CmdHandler handler, void *ctx);

/* 主循环调用：处理收到的命令，返回 true 表示参数已更新 */
bool BT_Poll(void);

/* 发送字符串（阻塞） */
void BT_Send(const char *str);

/* 格式化发送（阻塞） */
void BT_Printf(const char *fmt, ...);

/* 获取接收缓冲区指针（用于调试） */
const char *BT_GetRxBuf(void);

/* UART RX 中断处理（项目需在中断向量中调用） */
void BT_UartIrqHandler(void);

#endif /* MODULES_DRIVERS_BLUETOOTH_H */
