/**
 * @file  electromagnet.h
 * @brief 电磁铁 PWM 驱动 (SysConfig: PWM2 "EM" → TIMA1, PA17, 中心对齐)
 *
 * 磁力范围: 0.0 ~ 8.0 kg  (线性映射到 0~100% 占空比)
 *
 * @note  PWM 初始化由 SysConfig 生成的 SYSCFG_DL_init() 完成,
 *        本模块仅提供力控接口和 OLED 显示, 不配置任何硬件寄存器
 */

#ifndef __ELECTROMAGNET_H__
#define __ELECTROMAGNET_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ========== 参数定义 ========== */

/** @brief 电磁铁最大吸力 (kg) */
#define EM_MAX_FORCE_KG      8.0f

/**
 * @brief PWM 周期计数值 (必须与 SysConfig 中 PWM2 "EM" 的 timerCount 一致!)
 * @note  PWM 频率 = TimerClock / Period
 *        例: 32MHz / 1600 = 20kHz
 */
#define EM_PWM_PERIOD        1600U

/* ========== API ========== */

/**
 * @brief  初始化电磁铁状态 (占空比清零)
 * @note   不配置任何硬件寄存器, PWM 已由 SysConfig 初始化
 */
void EM_Init(void);

/**
 * @brief  设置电磁铁吸力 (kg)
 * @param  force_kg  目标吸力, 范围 0.0 ~ 8.0 kg, 超出自动钳位
 */
void EM_SetForce(float force_kg);

/**
 * @brief  设置电磁铁占空比 (%)
 * @param  percent  占空比, 范围 0 ~ 100, 超出自动钳位
 */
void EM_SetPercent(uint8_t percent);

/**
 * @brief  获取当前吸力 (kg)
 * @return 当前吸力值 (0.0 ~ 8.0)
 */
float EM_GetForce(void);

/**
 * @brief  获取当前占空比 (%)
 * @return 当前占空比 (0 ~ 100)
 */
uint8_t EM_GetPercent(void);

/**
 * @brief  在 OLED 上显示当前磁力值
 * @param  x    显示起始列 (0~127)
 * @param  y    显示行 (0~63)
 * @param  size 字体大小 (12 或 16)
 * @note   显示格式: "EM:X.Xkg"
 */
void EM_DisplayForce(uint8_t x, uint8_t y, uint8_t size);

#endif /* __ELECTROMAGNET_H__ */
