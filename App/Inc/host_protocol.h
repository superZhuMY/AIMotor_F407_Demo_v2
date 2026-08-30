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

/* 协议层初始化：启动 USART1 DMA 空闲接收（原 main.c 初始化代码迁入）。
   在 Aimotor_Init()/MW_Init() 之前调用。 */
void Host_ProtocolInit(void);

/* 非阻塞回复发送：ACK/STATE/文本回复统一走此通道（USART1 中断方式）。
   上一帧未发完时有界等待，异常时退化为阻塞发送。 */
void Host_ReplyBytes(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __HOST_PROTOCOL_H__ */
