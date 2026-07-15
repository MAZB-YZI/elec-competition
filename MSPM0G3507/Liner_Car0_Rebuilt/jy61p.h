/**
 * jy61p.h — JY61P 串口陀螺仪驱动
 *
 * 使用 UART0 (PA0=TX, PA1=RX)，波特率 9600
 * WIT 私有协议，角度帧: 0x55 0x53
 */

#ifndef __JY61P_H__
#define __JY61P_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化 JY61P（UART0 中断接收）
 */
void JY61P_Init(void);

/**
 * @brief UART0 中断处理（在 UART0_IRQHandler 中调用）
 */
void JY61P_UART_IRQHandler(void);

/**
 * @brief 获取 yaw 角度（单位：度，范围 -180 ~ +180）
 */
float JY61P_GetYaw(void);

/**
 * @brief 获取 yaw 原始值（2字节有符号整数）
 */
int16_t JY61P_GetYawRaw(void);

/**
 * @brief 获取 roll 角度（单位：度）
 */
float JY61P_GetRoll(void);

/**
 * @brief 获取 pitch 角度（单位：度）
 */
float JY61P_GetPitch(void);

/**
 * @brief 获取已接收的有效角度帧数量
 */
uint32_t JY61P_GetFrameCount(void);

/**
 * @brief 清零 yaw 偏移（当前角度设为零点）
 */
void JY61P_ZeroYaw(void);

/**
 * @brief 检查 JY61P 是否在线（最近 500ms 内收到过数据）
 */
bool JY61P_IsOnline(void);
void JY61P_UpdateTick(void);

#endif /* __JY61P_H__ */
