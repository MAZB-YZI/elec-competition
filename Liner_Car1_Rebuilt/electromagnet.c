/**
 * @file  electromagnet.c
 * @brief 电磁铁 PWM 驱动实现 (SysConfig "EM" → TIMA1 CCP0, 中心对齐)
 *
 * PWM 初始化由 SysConfig 生成的 SYSCFG_DL_init() 完成。
 * 本模块通过 EM_INST (SysConfig 自动生成) 操控占空比。
 *
 * 占空比映射 (中心对齐):
 *   0%  → compare = 0         (0 kg)
 *   50% → compare = PERIOD/2  (4 kg)
 *   100%→ compare = PERIOD    (8 kg)
 *
 * @note  实际磁力 F ∝ I² (感性负载), 当前使用线性映射
 */

#include "electromagnet.h"
#include "oled.h"

/* ========== 静态变量 ========== */

/** 当前占空比计数值 (0 ~ EM_PWM_PERIOD) */
static uint16_t g_em_duty = 0;

/* ========== 初始化 ========== */

void EM_Init(void)
{
    /* PWM 硬件已由 SYSCFG_DL_init() → EM_INST 配置完成, 此处仅置零 */
    DL_Timer_setCaptureCompareValue(EM_INST, 0, DL_TIMER_CC_0_INDEX);
    g_em_duty = 0;
}

/* ========== 力控制 ========== */

void EM_SetForce(float force_kg)
{
    if (force_kg < 0.0f)  force_kg = 0.0f;
    if (force_kg > EM_MAX_FORCE_KG) force_kg = EM_MAX_FORCE_KG;

    uint16_t duty = (uint16_t)((force_kg / EM_MAX_FORCE_KG) * EM_PWM_PERIOD + 0.5f);
    DL_Timer_setCaptureCompareValue(EM_INST, duty, DL_TIMER_CC_0_INDEX);
    g_em_duty = duty;
}

void EM_SetPercent(uint8_t percent)
{
    if (percent > 100) percent = 100;

    uint16_t duty = (uint16_t)(((float)percent / 100.0f) * EM_PWM_PERIOD + 0.5f);
    DL_Timer_setCaptureCompareValue(EM_INST, duty, DL_TIMER_CC_0_INDEX);
    g_em_duty = duty;
}

/* ========== 状态查询 ========== */

float EM_GetForce(void)
{
    return ((float)g_em_duty / EM_PWM_PERIOD) * EM_MAX_FORCE_KG;
}

uint8_t EM_GetPercent(void)
{
    return (uint8_t)(((uint32_t)g_em_duty * 100) / EM_PWM_PERIOD);
}

/* ========== OLED 显示 ========== */

void EM_DisplayForce(uint8_t x, uint8_t y, uint8_t size)
{
    float f = EM_GetForce();
    uint8_t int_part = (uint8_t)f;
    uint8_t dec_part = (uint8_t)((f - int_part) * 10.0f + 0.5f);

    OLED_ShowString(x, y, "EM:", size);
    x += (size == 16) ? 24 : 18;

    OLED_ShowNum(x, y, int_part, 1, size);
    x += (size == 16) ? 8 : 6;

    OLED_ShowString(x, y, ".", size);
    x += (size == 16) ? 8 : 6;

    OLED_ShowNum(x, y, dec_part, 1, size);
    x += (size == 16) ? 8 : 6;

    OLED_ShowString(x, y, "kg", size);
}
