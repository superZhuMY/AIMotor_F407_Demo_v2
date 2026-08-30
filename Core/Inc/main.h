/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* 干跑模式：1 时不向任何电机总线发送字节（仅保留上位机解析/ACK/STATE 路径），
   用于不驱动真机的回环测试。两个电机模块（aimotor.c / mwmotor.c）共用。
   默认测试模式下不向真实总线发送任何电机命令（包括 STOP/Servo Off）。 */
#ifndef AIMOTOR_DRY_RUN
#define AIMOTOR_DRY_RUN 0
#endif

/* 文本运动命令开关：0=发布/真机默认关闭所有会引起运动或使能的文本命令
   （B M P / B M HOME / B M GO / B M EN / B TEST / L/R J4 J5 J6 / L/R HOME）；
   只读命令（B M Q / L/R Q）与安全命令（STOP / ALL STOP / ALL DISABLE）始终可用。
   置 1 时文本运动命令必须经过与二进制相同的控制状态检查；B TEST 仍永久禁用，
   B M GO 改由生产状态机异步执行，不再阻塞等待。 */
#ifndef AIMOTOR_TEXT_MOTION_ENABLED
#define AIMOTOR_TEXT_MOTION_ENABLED 0
#endif

/* 固件自检：1 时要求同时开启 DRY_RUN，main 启动后执行 Aimotor_SelfTest()，
   直接调用生产 C 函数（解析器/状态机/换算/STATE/看门狗）并打印逐项结果。 */
#ifndef AIMOTOR_SELF_TEST
#define AIMOTOR_SELF_TEST 0
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
