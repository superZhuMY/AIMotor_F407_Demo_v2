/**
  ******************************************************************************
  * @file           : comm_router.c
  * @brief          : HAL UART 回调分发（原 main.c USER CODE 区迁入）
  *
  * 职责只有一件事：把 HAL 的 UART 事件按外设路由到对应模块。
  * USART1（上位机）字节流追加进协议层环形缓冲；AI/MW 总线回包由各自
  * 模块的 RxCallback 按从站号路由。本文件不做任何协议解析。
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "aimotor.h"
#include "aimotor_internal.h"
#include "host_protocol.h"
#include "mwmotor.h"

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
