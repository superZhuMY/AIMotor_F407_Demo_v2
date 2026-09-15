/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : aimotor_modbus.c
  * @brief          : AI Motor Modbus RTU 编解码与 RS485 发送层
  *                   （自 aimotor.c 拆分，函数体逐行搬迁未改动）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "aimotor.h"

/* ======================================================================== */
/*                     Int32 CDAB 编解码辅助函数                               */
/* ======================================================================== */

static void Aimotor_EncodeInt32CDAB(int32_t value, uint8_t out[4])
{
    /* int32_t 转 uint32_t 的结果按模 2^32 转换，可稳定得到补码位模式 */
    uint32_t raw = (uint32_t)value;

    uint16_t low_word  = (uint16_t)(raw & 0xFFFFU);
    uint16_t high_word = (uint16_t)((raw >> 16) & 0xFFFFU);

    out[0] = (uint8_t)(low_word >> 8);
    out[1] = (uint8_t)(low_word & 0xFFU);
    out[2] = (uint8_t)(high_word >> 8);
    out[3] = (uint8_t)(high_word & 0xFFU);
}

int32_t Aimotor_DecodeInt32CDAB(const uint8_t data[4])
{
    uint16_t low_word =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    uint16_t high_word =
        ((uint16_t)data[2] << 8) |
        (uint16_t)data[3];

    uint32_t raw =
        ((uint32_t)high_word << 16) |
        (uint32_t)low_word;

    /* 显式按32位补码解释，避免有符号移位和超范围强转依赖 */
    if ((raw & 0x80000000U) != 0U) {
        int64_t signed_value = (int64_t)raw - 0x100000000LL;
        return (int32_t)signed_value;
    }

    return (int32_t)raw;
}

/* ======================================================================== */
/*                          CRC16 计算                                       */
/* ======================================================================== */

uint16_t Aimotor_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ======================================================================== */
/*                         RS485 硬件发送                                    */
/* ======================================================================== */

void Aimotor_BusSendBytes(uint8_t bus_idx, uint16_t len)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) return;

    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

#if AIMOTOR_DRY_RUN
    /* 干跑模式：AI 总线不发送任何命令。真实发送计数 g_dry_run_ai_tx_count
       恒为 0（HAL 发送代码被编译掉）；待发送帧由 mock 记录。
       参数/状态检查在调用方已全部执行。 */
    (void)len;
    Aimotor_DryRunLog(0, bus->tx_buf, len);
    return;
#else
    /* TX LED 亮 */
    if (bus->led_port) {
        HAL_GPIO_WritePin(bus->led_port, bus->led_pin, GPIO_PIN_SET);
    }

    if (HAL_UART_Transmit(bus->huart, bus->tx_buf, len, AIMOTOR_DMA_TIMEOUT) != HAL_OK) {
        if (bus->led_port)
            HAL_GPIO_WritePin(bus->led_port, bus->led_pin, GPIO_PIN_RESET);
        return;
    }
    g_dry_run_ai_tx_count++;   /* 真实发送计数（DRY_RUN 下永不执行，恒为 0） */

    /* HAL_UART_Transmit 返回即表示最后一个停止位已送出（内部已等待 TC），
       此处不再重复等待 UART_FLAG_TC。 */

    /* TX LED 灭 */
    if (bus->led_port) {
        HAL_GPIO_WritePin(bus->led_port, bus->led_pin, GPIO_PIN_RESET);
    }
#endif
}

/* ======================================================================== */
/*                   Modbus 命令生成函数（参数化 bus_idx）                       */
/* ======================================================================== */

/* 伺服使能：写 H03_03 = 1 */
void Aimotor_ServoOn(uint8_t bus_idx, uint8_t slave_id)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) return;
    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

    bus->tx_buf[0] = slave_id;
    bus->tx_buf[1] = 0x06;        /* 写单个寄存器 */
    bus->tx_buf[2] = 0x03;        /* H03 */
    bus->tx_buf[3] = 0x03;        /* _03 (0x0303) */
    bus->tx_buf[4] = 0x00;
    bus->tx_buf[5] = 0x01;        /* 写入1: 电机使能导通 */

    uint16_t crc = Aimotor_CRC16(bus->tx_buf, 6);
    bus->tx_buf[6] = (uint8_t)(crc & 0xFF);
    bus->tx_buf[7] = (uint8_t)(crc >> 8);
    Aimotor_BusSendBytes(bus_idx, 8);
}

/* 伺服断开：写 H03_03 = 0（与 ServoOn 同一寄存器，逆值）。
   应答为 0x06 写单寄存器回包，由控制序列应答校验确认。 */
void Aimotor_ServoOff(uint8_t bus_idx, uint8_t slave_id)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) return;
    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

    bus->tx_buf[0] = slave_id;
    bus->tx_buf[1] = 0x06;        /* 写单个寄存器 */
    bus->tx_buf[2] = 0x03;        /* H03 */
    bus->tx_buf[3] = 0x03;        /* _03 (0x0303) */
    bus->tx_buf[4] = 0x00;
    bus->tx_buf[5] = 0x00;        /* 写入0: 伺服断开 */

    uint16_t crc = Aimotor_CRC16(bus->tx_buf, 6);
    bus->tx_buf[6] = (uint8_t)(crc & 0xFF);
    bus->tx_buf[7] = (uint8_t)(crc >> 8);
    Aimotor_BusSendBytes(bus_idx, 8);
}

/* 停止多段位运行：10H 写 H03_05 = 0 (与CPP完全一致) */
void Aimotor_StopMotion(uint8_t bus_idx, uint8_t slave_id)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) return;
    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

    bus->tx_buf[0] = slave_id;
    bus->tx_buf[1] = 0x10;        /* 写多寄存器 */
    bus->tx_buf[2] = 0x03;        /* H03 */
    bus->tx_buf[3] = 0x05;        /* _05 (0x0305) */
    bus->tx_buf[4] = 0x00;
    bus->tx_buf[5] = 0x01;        /* 写1个寄存器 */
    bus->tx_buf[6] = 0x02;        /* 2字节 */
    bus->tx_buf[7] = 0x00;
    bus->tx_buf[8] = 0x00;        /* 值=0 */

    uint16_t crc = Aimotor_CRC16(bus->tx_buf, 9);
    bus->tx_buf[9]  = (uint8_t)(crc & 0xFF);
    bus->tx_buf[10] = (uint8_t)(crc >> 8);
    Aimotor_BusSendBytes(bus_idx, 11);
}

/* 写位置：10H -> 0x110C，仅写2个寄存器（位置），不写速度 */
void Aimotor_SendPosition(uint8_t bus_idx, uint8_t slave_id,
                          int32_t pulse)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) {
        return;
    }

    if (pulse < AIMOTOR_POSITION_MIN ||
        pulse > AIMOTOR_POSITION_MAX) {
        return;
    }

    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

    bus->tx_buf[0] = slave_id;
    bus->tx_buf[1] = 0x10;
    bus->tx_buf[2] = 0x11;
    bus->tx_buf[3] = 0x0C;
    bus->tx_buf[4] = 0x00;
    bus->tx_buf[5] = 0x02;  /* 只写2个寄存器 */
    bus->tx_buf[6] = 0x04;  /* 位置共4字节 */

    Aimotor_EncodeInt32CDAB(pulse, &bus->tx_buf[7]);

    uint16_t crc = Aimotor_CRC16(bus->tx_buf, 11);
    bus->tx_buf[11] = (uint8_t)(crc & 0xFFU);
    bus->tx_buf[12] = (uint8_t)(crc >> 8);

    Aimotor_BusSendBytes(bus_idx, 13);
}

/* 触发运行：10H 写 H03_05 = 1 (与CPP完全一致) */
void Aimotor_TriggerMotion(uint8_t bus_idx, uint8_t slave_id)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) return;
    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

    bus->tx_buf[0] = slave_id;
    bus->tx_buf[1] = 0x10;        /* 写多寄存器 */
    bus->tx_buf[2] = 0x03;        /* H03 */
    bus->tx_buf[3] = 0x05;        /* _05 (0x0305) */
    bus->tx_buf[4] = 0x00;
    bus->tx_buf[5] = 0x01;        /* 写1个寄存器 */
    bus->tx_buf[6] = 0x02;        /* 2字节 */
    bus->tx_buf[7] = 0x00;
    bus->tx_buf[8] = 0x01;        /* 值=1 */

    uint16_t crc = Aimotor_CRC16(bus->tx_buf, 9);
    bus->tx_buf[9]  = (uint8_t)(crc & 0xFF);
    bus->tx_buf[10] = (uint8_t)(crc >> 8);
    Aimotor_BusSendBytes(bus_idx, 11);
}

/* 查询位置：03H -> 0x0B07（读 2 个寄存器） */
void Aimotor_QueryPosition(uint8_t bus_idx, uint8_t slave_id)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) return;
    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

    bus->tx_buf[0] = slave_id;
    bus->tx_buf[1] = 0x03;        /* 读寄存器 */
    bus->tx_buf[2] = 0x0B;        /* H0B */
    bus->tx_buf[3] = 0x07;        /* _07 (0x0B07) */
    bus->tx_buf[4] = 0x00;
    bus->tx_buf[5] = 0x02;        /* 读2个寄存器 */

    uint16_t crc = Aimotor_CRC16(bus->tx_buf, 6);
    bus->tx_buf[6] = (uint8_t)(crc & 0xFF);
    bus->tx_buf[7] = (uint8_t)(crc >> 8);
    Aimotor_BusSendBytes(bus_idx, 8);
}

/* 查询H0B_15：编码器位置偏差计数器（32位） */
void Aimotor_QueryPositionError(uint8_t bus_idx, uint8_t slave_id)
{
    if (bus_idx >= AIMOTOR_BUS_COUNT) return;
    Aimotor_Bus_t *bus = &aimotor_buses[bus_idx];

    bus->tx_buf[0] = slave_id;
    bus->tx_buf[1] = 0x03;
    bus->tx_buf[2] = 0x0B;
    bus->tx_buf[3] = 0x15;
    bus->tx_buf[4] = 0x00;
    bus->tx_buf[5] = 0x02;

    uint16_t crc = Aimotor_CRC16(bus->tx_buf, 6);
    bus->tx_buf[6] = (uint8_t)(crc & 0xFFU);
    bus->tx_buf[7] = (uint8_t)(crc >> 8);
    Aimotor_BusSendBytes(bus_idx, 8);
}
