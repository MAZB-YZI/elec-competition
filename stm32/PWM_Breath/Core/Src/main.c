/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Includes */
#include "string.h"
// 提供 strcmp() 函数
// 用来比较字符串是否等于"Auto"

#include "stdlib.h"
// 提供 atoi() 函数
// 用来把字符串"50"转换成数字50
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t  rx_data  = 0;     // 串口每次接收1个字节存这里
uint8_t  rx_buf[20];       // 拼接完整命令字符串，比如"Auto"或"50"
uint8_t  rx_idx   = 0;     // 记录当前存到buf的第几个位置
uint8_t  breath_en = 0;    // 呼吸灯开关：0=手动调光  1=呼吸灯
int16_t  breath_val = 0;   // 呼吸灯当前占空比 0~99
int8_t   breath_dir = 1;   // 呼吸方向：1=变亮  -1=变暗
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
// 启动TIM1的PWM输出
// 没有这句PE9不会有任何信号输出

HAL_TIM_Base_Start_IT(&htim6);
// 启动TIM6定时中断
// IT = Interrupt，带中断的启动
// 没有这句呼吸灯定时器不工作

HAL_UART_Receive_IT(&huart1, &rx_data, 1);
// 启动串口中断接收
// 每次接收1个字节存到rx_data
// 没有这句串口收不到任何数据

__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
// 设置初始占空比50%
// LED上电就是半亮状态
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// 函数名解释：
// HAL_UART  = HAL库的串口函数
// RxCplt    = Receive Complete 接收完成
// Callback  = 回调函数，中断触发后自动调用
// *huart    = 指针，告诉你是哪个串口触发的
{
    if (huart->Instance == USART1)
    // huart->Instance = 当前触发的串口号
    // 判断是不是USART1触发的
    // 因为板子可能有多个串口
    {
        if (rx_data == '\n' || rx_data == '\r' || rx_idx >= 19)
        // '\n' = 换行符  '\r' = 回车符
        // 收到换行说明一条命令发完了
        // rx_idx>=19 防止缓冲区溢出
        {
            rx_buf[rx_idx] = '\0';
            // '\0' = 字符串结束符
            // 告诉系统字符串到这里结束了

            if (strcmp((char*)rx_buf, "Auto") == 0 ||
                strcmp((char*)rx_buf, "auto") == 0)
            // strcmp = string compare 字符串比较
            // 返回0说明两个字符串相同
            // 同时支持大小写Auto和auto
            {
                breath_val = 0;
                breath_dir = 1;
                breath_en  = 1;
                // 从0开始变亮，开启呼吸灯模式
            }
            else
            {
                int val = atoi((char*)rx_buf);
                // atoi = ascii to integer
                // 把字符串"50"转换成数字50
                
                if (val >= 0 && val <= 100)
                {
                    breath_en = 0;
                    // 关闭呼吸灯，切换到手动模式
                    
                    if (val == 100) val = 99;
                    // ARR=99，最大只能填99
                    // 填100会超出范围
                    
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, val);
                    // 直接设置占空比
                    // SET_COMPARE = 设置比较值CCR1
                }
            }
            rx_idx = 0;
            // 清空缓冲区，准备接收下一条命令
        }
        else
        {
            rx_buf[rx_idx++] = rx_data;
            // 把这个字节存入缓冲区
            // rx_idx++ 索引加1，指向下一个位置
        }

        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
        // ⚠️ 非常重要！
        // 串口中断是一次性的，用完就没了
        // 必须重新开启，才能接收下一个字节
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// HAL_TIM        = HAL库定时器函数
// PeriodElapsed  = 周期结束，即计数溢出
// Callback       = 回调函数
// *htim          = 哪个定时器触发的
{
    if (htim->Instance == TIM6)
    // 判断是TIM6触发的
    // TIM1也有中断，要区分开
    {
        if (breath_en == 1)
        // 只有呼吸灯模式才执行
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                                  (uint32_t)breath_val);
            // 把当前占空比写入TIM1
            // uint32_t = 强制转换为无符号整数
            // 因为breath_val是int16_t有符号的

            breath_val += breath_dir;
            // breath_dir=1  → breath_val+1 变亮
            // breath_dir=-1 → breath_val-1 变暗

            if (breath_val >= 99)
            {
                breath_val = 99;
                breath_dir = -1;
                // 到顶了，开始变暗
            }
            else if (breath_val <= 0)
            {
                breath_val = 0;
                breath_dir = 1;
                // 到底了，开始变亮
            }
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
