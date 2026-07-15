/* ================================================================
 * 把下面的代码分别粘贴到对应位置
 * ================================================================ */


/* ────────────────────────────────────────────────────────────────
 * 1. 粘贴到 stm32f4xx_it.c 顶部（#include 区域之后）
 * ────────────────────────────────────────────────────────────────
 */
// #include "board_b.h"   // 在 it.c 里加这一行


/* ────────────────────────────────────────────────────────────────
 * 2. 粘贴到 main.c 的 USER CODE BEGIN Includes 区域
 * ────────────────────────────────────────────────────────────────
 */
// #include "board_b.h"


/* ────────────────────────────────────────────────────────────────
 * 3. 粘贴到 main.c 的 USER CODE BEGIN 2（外设初始化完成后）
 * ────────────────────────────────────────────────────────────────
 */
// BoardB_Init();


/* ────────────────────────────────────────────────────────────────
 * 4. 粘贴到 main.c 的 while(1) 循环体（USER CODE BEGIN 3）
 * ────────────────────────────────────────────────────────────────
 */
// BoardB_MainLoop();


/* ────────────────────────────────────────────────────────────────
 * 5. 在 main.c 末尾（USER CODE BEGIN 4）添加以下三个回调函数
 *    HAL 库会自动调用 weak 版本，这里覆盖它们
 * ────────────────────────────────────────────────────────────────
 */

/* TIM6(1ms) 和 TIM7(100ms) 共用 PeriodElapsedCallback
   HAL 用 htim 指针区分是哪个定时器 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    extern TIM_HandleTypeDef htim6, htim7;

    if (htim->Instance == TIM6) {
        BoardB_TIM6_1ms_Callback();
    }
    else if (htim->Instance == TIM7) {
        BoardB_TIM7_100ms_Callback();
    }

    /* 注意：如果 CubeMX 把 SysTick 的 HAL_IncTick 也放在这个回调里，
       需要保留 htim == &htimX 的判断，不要误触发。
       CubeMX 默认用 SysTick 驱动 HAL tick，不会走这个回调，放心使用。 */
}

/* UART IDLE 事件 + DMA 接收完成回调（HAL LL 2.x 统一接口）*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    extern UART_HandleTypeDef huart1;

    if (huart->Instance == USART1) {
        BoardB_UART_RxCallback(Size);
    }
}

/* ────────────────────────────────────────────────────────────────
 * 6. CubeMX 生成的 MX_TIM1_Init 里，在最后添加：
 *    确保 MOE（主输出使能）位被打开
 *    （如果 CubeMX 里勾了 "Automatic Output Enable"，此行可省略）
 * ────────────────────────────────────────────────────────────────
 */
/*
   在 MX_TIM1_Init() 末尾：
   __HAL_TIM_MOE_ENABLE(&htim1);   // 确保互补输出有效
*/


/* ────────────────────────────────────────────────────────────────
 * 7. CubeMX 死区配置备注
 *
 *    在 TIM1 的 "Break and Dead-time" 选项卡：
 *      Dead Time  = 100
 *      (对应约 595ns 死区 @ 168MHz，示波器双踪清晰可见)
 *
 *    如果想在代码里手动设置，在 MX_TIM1_Init() 后写：
 *      TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG_Msk) | 100u;
 *      TIM1->BDTR |= TIM_BDTR_MOE;
 * ────────────────────────────────────────────────────────────────
 */
