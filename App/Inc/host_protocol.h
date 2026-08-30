/**
  ******************************************************************************
  * @file           : host_protocol.h
  * @brief          : 上位机协议层（USART1）公共接口：初始化与回复通道
  ******************************************************************************
  */

#ifndef __HOST_PROTOCOL_H__
#define __HOST_PROTOCOL_H__

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  协议层初始化：启动 USART1 DMA 空闲接收（ISR 只追加字节，
 *         解析在主循环 Aimotor_HostStreamPoll 中完成）
 * @note   在 Aimotor_Init()/MW_Init() 之前、主循环之前调用一次
 */
void Host_ProtocolInit(void);

/**
 * @brief  非阻塞回复发送：ACK/STATE/文本回复统一通道（USART1 中断方式）
 *         上一帧未发完时有界等待（20ms），异常时退化为阻塞发送
 * @param  data 回复缓冲（发送完成前调用方不得改写/释放）
 * @param  len  字节数
 * @note   仅主循环上下文调用；自检输出仍走阻塞发送（运行于主循环前）
 */
void Host_ReplyBytes(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __HOST_PROTOCOL_H__ */
