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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "aimotor.h"
#include "mwmotor.h"
#include <string.h>
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

uint32_t aimotor_last_tick = 0;
uint32_t mwmotor_last_tick = 0;

/* 分离的上位机缓冲：DMA接收、命令解析、发送回复 */
uint8_t g_host_dma_buf[64];
char    g_host_cmd_buf[64];   /* 命令缓冲（流解析填充，aimotor.c 消费） */
char    g_host_tx_buf[96];

/* 命令缓冲由 aimotor.c 的流解析（环形缓冲）填充，主循环消费 */
volatile uint16_t g_host_cmd_len = 0;
volatile uint8_t  g_host_cmd_ready = 0;

/* 旧名称兼容别名（供 mwmotor.c 等模块引用，不直接使用） */
uint8_t            g_host_rx_buf[64];
volatile uint16_t  g_host_rx_len = 0;
volatile uint8_t   g_host_rx_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  DMA + IDLE 空闲中断回调路由
  *         按 USART 外设分发到对应模块
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* USART1: 上位机字节流 — 只把本次收到的字节追加入环形缓冲，
       拆包/粘包/CRC 校验/文本行提取全部在主循环的流解析中完成。 */
    if (huart->Instance == USART1) {
        if (Size > sizeof(g_host_dma_buf)) {
            Size = sizeof(g_host_dma_buf);
        }
        Aimotor_HostRxAppend(g_host_dma_buf, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_host_dma_buf,
                                     sizeof(g_host_dma_buf));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        return;
    }

    /* AI 电机总线 (USART2=左, USART3=右) */
    Aimotor_RxCallback(huart, Size);

    /* 慕纬度电机总线 (USART6=左, UART5=右) */
    MW_RxCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_AbortReceive(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_host_dma_buf,
                                     sizeof(g_host_dma_buf));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        return;
    }

    Aimotor_RecoverRx(huart);
    MW_RecoverRx(huart);
}

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

		/* 启动上位机 USART1 DMA 空闲接收 */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_host_dma_buf, sizeof(g_host_dma_buf));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
		/* 初始化 AI 电机系统（USART2/USART3, Modbus RTU） */
		Aimotor_Init();
		/* 初始化慕纬度电机系统（USART6/UART5, 私有协议） */
		MW_Init();

#if AIMOTOR_SELF_TEST
		/* 固件自检：直接调用生产 C 函数（解析器/状态机/换算/STATE/看门狗），
		   输出逐项 PASS/FAIL 到 USART1。仅在 DRY_RUN=1 时允许。 */
		Aimotor_SelfTest();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* 两类电机状态机均按 5ms 调度；实际发送由“一次一帧、等待回包”限制。 */
    uint32_t now = HAL_GetTick();
    if (now - aimotor_last_tick >= AIMOTOR_TICK_MS) {
        aimotor_last_tick = now;
        Aimotor_Process();      /* 内含上位机流解析 */
        Aimotor_CommWatchdog(); /* 通信看门狗（无阻塞） */
    }

    /* J4/J5/J6独立使用5ms周期，缩短耦合轴命令间隔。 */
    if (now - mwmotor_last_tick >= MW_TICK_MS) {
        mwmotor_last_tick = now;
        MW_Process();
    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL return error state */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, file) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
