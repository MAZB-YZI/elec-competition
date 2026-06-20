/* ================================================================
 * A板接入说明 — 粘贴到 main.c 对应位置
 * ================================================================ */

/* ── USER CODE BEGIN Includes ── */
// #include "board_a.h"

/* ── USER CODE BEGIN 2（所有 MX_Init 完成后）── */
// BoardA_Init();

/* ── USER CODE BEGIN 3（while(1) 里）── */
// BoardA_MainLoop();

/* ── USER CODE BEGIN 4（回调函数）── */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        BoardA_TIM6_1ms_Callback();
    }
    else if (htim->Instance == TIM7) {
        BoardA_TIM7_100ms_Callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        BoardA_UART_RxCallback(Size);
    }
}

/* ================================================================
 * A板 CubeMX 配置与 B板完全一致，唯一区别：
 *   - 不需要配置 TIM1（A板没有互补PWM）
 *   - TIM3 CH3 驱动 D1 呼吸灯（同B板的D2配置）
 *   - 其余 TIM2/TIM6/TIM7/ADC1/USART1/I2C1/GPIO 完全相同
 *
 * 注意：board_a.c 里用了 math.h 的 sinf()
 *   Keil 需要在 Target Options → C/C++ → Misc Controls 加：
 *   --gnu  或者在 Link 选项加 math 库
 *   如果编译报 sinf 未定义，在 main.c 顶部加：
 *   #include <math.h>
 *   并在 Keil Options → Target 勾选 Use MicroLIB（或不勾，用标准库）
 * ================================================================ */
