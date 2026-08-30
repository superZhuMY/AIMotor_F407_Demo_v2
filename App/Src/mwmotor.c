/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mwmotor.c
  * @brief          : Moweidu motor private protocol (0x3E) module
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mwmotor.h"
#include "aimotor.h"   /* Aimotor_RefreshHostWatchdog() */
#include "aimotor_internal.h"   /* g_mw_cmd_* 命令槽 */
#include "host_protocol.h"   /* Host_ReplyBytes() */
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* USER CODE BEGIN 0 */

/* ======================================================================== */
/*                          全局变量定义                                      */
/* ======================================================================== */

MW_Bus_t   mw_buses[MW_BUS_COUNT];
MW_Motor_t mw_motors[MW_BUS_COUNT][MW_MOTORS_PER];
MW_CoupledTask_t mw_coupled[MW_BUS_COUNT];

/* 空闲位置轮询：每个总线任一时刻只有一个明确的读取事务所有者 */
typedef enum {
    MW_IDLE_NONE = 0,
    MW_IDLE_READ_J4,   /* 0x92 ID4（J4 独立轴） */
    MW_IDLE_READ_J5,   /* 0x92 ID5（J5/J6 耦合事务第一步） */
    MW_IDLE_READ_J6    /* 0x92 ID6（J5/J6 耦合事务第二步） */
} MwIdleReadState_t;

static uint8_t mw_idle_poll_cnt[MW_BUS_COUNT];
static MwIdleReadState_t mw_idle_owner[MW_BUS_COUNT];
/* 下一轮空闲读取目标：J4 与 J5->J6 整轮交替。 */
static MwIdleReadState_t mw_idle_next[MW_BUS_COUNT];

/* 上位机 L/R 文本命令槽：host_protocol.c 的流解析填充，经 aimotor_internal.h 共享 */
/* MW 文本回复缓冲：中断发送要求缓冲在发送期间保持稳定，不能复用命令槽 */
static char g_mw_reply_buf[96];

/* 静态函数 */
static void ProcessRx(uint8_t bus_idx, uint8_t motor_idx);
static void MW_CmdParse(void);
static int64_t MW_ReadI64LE(const uint8_t *data);
static uint8_t MW_CoupledTimedOut(const MW_CoupledTask_t *task);

/* ======================================================================== */
/*                          和校验（低8位）                                   */
/* ======================================================================== */

uint8_t MW_Checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += buf[i];
    return sum;
}

/* ======================================================================== */
/*                          RS485 发送 + LED                                 */
/* ======================================================================== */

void MW_SendBytes(uint8_t bus_idx, uint16_t len)
{
    if (bus_idx >= MW_BUS_COUNT) return;
    MW_Bus_t *bus = &mw_buses[bus_idx];

#if AIMOTOR_DRY_RUN
    /* 干跑模式：MW 总线不发送任何命令。真实发送计数 g_dry_run_mw_tx_count
       恒为 0（HAL 发送代码被编译掉）；待发送帧由 mock 记录。 */
    (void)len;
    Aimotor_DryRunLog(1, bus->tx_buf, len);
    return;
#else
    if (bus->led_port)
        HAL_GPIO_WritePin(bus->led_port, bus->led_pin, GPIO_PIN_SET);

    if (HAL_UART_Transmit(bus->huart, bus->tx_buf, len, MW_TIMEOUT_MS) != HAL_OK) {
        if (bus->led_port)
            HAL_GPIO_WritePin(bus->led_port, bus->led_pin, GPIO_PIN_RESET);
        return;
    }
    g_dry_run_mw_tx_count++;   /* 真实发送计数（DRY_RUN 下永不执行，恒为 0） */
    uint32_t start = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start) >= MW_TIMEOUT_MS)
            break;
    }

    if (bus->led_port)
        HAL_GPIO_WritePin(bus->led_port, bus->led_pin, GPIO_PIN_RESET);
#endif
}

/* ======================================================================== */
/*                   0xA3 多圈位置控制命令1（ROS现用协议）                    */
/* ======================================================================== */

void MW_SendA3(uint8_t bus_idx, uint8_t id, int64_t angle)
{
    MW_Bus_t *bus = &mw_buses[bus_idx];
    uint8_t *buf = bus->tx_buf;

    buf[0] = 0x3E;                    // 帧头
    buf[1] = 0xA3;                    // 多圈位置控制命令1
    buf[2] = id;                      // ID
    buf[3] = 0x08;                    // 数据长度 8
    buf[4] = MW_Checksum(buf, 4);     // CMD_SUM

    // int64_t 小端 8字节
    for (int i = 0; i < 8; i++)
        buf[5 + i] = (uint8_t)(((uint64_t)angle) >> (i * 8));

    buf[13] = MW_Checksum(buf + 5, 8);  // DATA_SUM

    MW_SendBytes(bus_idx, 14);
}

/* ======================================================================== */
/*                   0x92 读取多圈角度                                        */
/* ======================================================================== */

void MW_Send92(uint8_t bus_idx, uint8_t id)
{
    MW_Bus_t *bus = &mw_buses[bus_idx];
    uint8_t *buf = bus->tx_buf;

    buf[0] = 0x3E;
    buf[1] = 0x92;
    buf[2] = id;
    buf[3] = 0x00;
    buf[4] = MW_Checksum(buf, 4);

    MW_SendBytes(bus_idx, 5);
}

/* ======================================================================== */
/*                   0x80 电机关闭                                            */
/* ======================================================================== */

void MW_Send80(uint8_t bus_idx, uint8_t id)
{
    MW_Bus_t *bus = &mw_buses[bus_idx];
    uint8_t *buf = bus->tx_buf;

    buf[0] = 0x3E;
    buf[1] = 0x80;
    buf[2] = id;
    buf[3] = 0x00;
    buf[4] = MW_Checksum(buf, 4);

    MW_SendBytes(bus_idx, 5);
}

/* ======================================================================== */
/*                   0x88 电机运行                                            */
/* ======================================================================== */

void MW_Send88(uint8_t bus_idx, uint8_t id)
{
    MW_Bus_t *bus = &mw_buses[bus_idx];
    uint8_t *buf = bus->tx_buf;

    buf[0] = 0x3E;
    buf[1] = 0x88;
    buf[2] = id;
    buf[3] = 0x00;
    buf[4] = MW_Checksum(buf, 4);

    MW_SendBytes(bus_idx, 5);
}

/* ======================================================================== */
/*                       回包解析处理                                        */
/* ======================================================================== */

static void ProcessRx(uint8_t bus_idx, uint8_t motor_idx)
{
    MW_Bus_t *bus = &mw_buses[bus_idx];
    MW_Motor_t *motor = &mw_motors[bus_idx][motor_idx];

    bus->rx_ready = 0;

    uint8_t *rx = bus->rx_buf;

    /* 校验帧头 */
    if (rx[0] != 0x3E) return;

    /* 校验 ID 匹配 */
    if (rx[2] != motor->id) return;

    /* 校验 CMD_SUM */
    uint8_t cmd_sum = MW_Checksum(rx, 4);
    if (cmd_sum != rx[4]) return;

    uint8_t data_len = rx[3];

    switch (rx[1]) {

        case 0x92: {
            /* 读取多圈角度回复：14 字节 */
            if (data_len != 0x08 || bus->rx_len < 14) break;

            uint8_t data_sum = MW_Checksum(rx + 5, 8);
            if (data_sum != rx[5 + 8]) break;  // DATA_SUM 在 data 末尾

            /* int64_t 小端拼接 */
            motor->actual_angle = MW_ReadI64LE(&rx[5]);
            motor->actual_valid = 1;

            /* 只有合法回包成功后才清零重试计数 */
            motor->retry_count = 0;
            motor->step = MW_STEP_IDLE;  // 读取完成，回到空闲
            break;
        }

        case 0xA3:
        case 0x88:
        case 0x80: {
            /* A3回复应为状态2格式：命令头5字节+数据7字节+DATA_SUM */
            if (rx[1] == 0xA3 && (data_len != 0x07 || bus->rx_len < 13)) {
                break;
            }
            if (data_len > 0 &&
                (uint16_t)(5U + data_len + 1U) > bus->rx_len) {
                break;
            }
            if (data_len > 0 &&
                MW_Checksum(rx + 5, data_len) != rx[5 + data_len]) {
                break;
            }
            /* 只有合法回包成功后才清零重试计数 */
            motor->retry_count = 0;
            motor->step = MW_STEP_IDLE;
            break;
        }

        default:
            break;
    }
}

/* ======================================================================== */
/*                       上位机命令解析与执行 (L/R prefix)                   */
/* ======================================================================== */

/**
  * 格式:
  *   L J5 <deg> J6 <deg> S <speed>   — 左臂 J5/J6 角度, 速度
  *   R J5 <deg> J6 <deg> S <speed>   — 右臂 J5/J6 角度, 速度
  *   L STOP                           — 左臂停止
  *   R STOP                           — 右臂停止
  */
static void MW_CmdParse(void)
{
    const uint8_t *buf = g_mw_cmd_buf;
    uint16_t len = g_mw_cmd_len;

    if (len < 4) return;

    uint8_t side;

    /* 解析 L 或 R */
    if (buf[0] == 'L' || buf[0] == 'l') side = 0;
    else if (buf[0] == 'R' || buf[0] == 'r') side = 1;
    else return;

    /* L/R STOP：文本停止统一走异步全局安全停止序列（非阻塞、回包确认），
       不再连续发送 3 个 0x80 并直接清零 enabled。 */
    if (strstr((const char *)buf, "STOP") || strstr((const char *)buf, "stop")) {
        Aimotor_ControlStopStart();
        int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                         "OK STOP: ACCEPTED\r\n");
        Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
        return;
    }

    /* L/R Q：只读查询，始终可用 */
    if (strstr((const char *)buf, "Q") || strstr((const char *)buf, "q")) {
        int64_t angles[3];
        for (uint8_t m = 0; m < MW_MOTORS_PER; m++) {
            angles[m] = mw_motors[side][m].actual_angle;
        }

        /* 当前ROS兼容模式保留1000计数/度的换算。 */
        int16_t j4_deg = (int16_t)(angles[0] / 1000);
        int16_t j5_deg = (int16_t)(angles[1] * 3 / 5000);
        if (side == 1) j5_deg = (int16_t)-j5_deg;
        int16_t j6_deg = (int16_t)((angles[2] + angles[1]) * 9 / 20000);

        int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                         "POS %c J4=%d J5=%d J6=%d\r\n",
                         side == 0 ? 'L' : 'R', j4_deg, j5_deg, j6_deg);
        Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
        return;
    }

#if !AIMOTOR_TEXT_MOTION_ENABLED
    /* 发布配置：运动类文本命令（HOME / J4 J5 J6）默认关闭 */
    if (strstr((const char *)buf, "HOME") || strstr((const char *)buf, "home") ||
        strstr((const char *)buf, "J4") || strstr((const char *)buf, "J5") ||
        strstr((const char *)buf, "J6") || strstr((const char *)buf, "j4") ||
        strstr((const char *)buf, "j5") || strstr((const char *)buf, "j6")) {
        int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                         "ERR TEXT MOTION DISABLED\r\n");
        Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
        return;
    }
    return;
#else
    /* 调试配置：文本运动命令必须经过与二进制相同的控制状态检查 */
    if (Aimotor_GetControlState() != CONTROL_ENABLED) {
        int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                         "ERR CONTROL DENIED\r\n");
        Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
        return;
    }
    if (mw_coupled[side].fault || mw_motors[side][0].fault ||
        mw_motors[side][1].fault || mw_motors[side][2].fault) {
        int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                         "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
        Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
        return;
    }
#endif
    if (strstr((const char *)buf, "HOME") || strstr((const char *)buf, "home")) {
        uint32_t spd = 18000; /* 回零默认速度 180°/s */
        const char *q = strstr((const char *)buf, "S");
        if (!q) q = strstr((const char *)buf, "s");
        if (q) { spd = (uint32_t)atoi(q + 1) * 100; if (spd > 3600000) spd = 3600000; }

        /* 检查是否指定了具体关节 */
        uint8_t has_j4 = (strstr((const char *)buf, "J4") || strstr((const char *)buf, "j4"));
        uint8_t has_j5 = (strstr((const char *)buf, "J5") || strstr((const char *)buf, "j5"));
        uint8_t has_j6 = (strstr((const char *)buf, "J6") || strstr((const char *)buf, "j6"));
        uint8_t any_j = has_j4 || has_j5 || has_j6;

        for (uint8_t m = 0; m < MW_MOTORS_PER; m++) {
            uint8_t match = (!any_j) ||                           /* L HOME → 全部 */
                           (has_j4 && m == 0) ||                   /* L J4 HOME */
                           (has_j5 && m == 1) ||                   /* L J5 HOME */
                           (has_j6 && m == 2);                     /* L J6 HOME */
            if (match) {
                if (m == 0) {
                    mw_motors[side][m].target_angle = 0;
                    mw_motors[side][m].target_speed = spd;
                    mw_motors[side][m].cmd_pending = 1;
                }
            }
        }
        if (!any_j || has_j5 || has_j6) {
            mw_coupled[side].requested_j5_deg1000 = 0;
            mw_coupled[side].requested_j6_deg1000 = 0;
            mw_coupled[side].pending_motor5_target = 0;
            mw_coupled[side].pending_motor6_target = 0;
            mw_coupled[side].pending = 1;
        }

        int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                         "OK %c HOME\r\n", side == 0 ? 'L' : 'R');
        Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
        return;
    }

    /* ALL STOP 由 HostCmd_Parse 处理 */

    /* 解析 J4 J5 J6 和 S；未提供的关节保持上一次目标。 */
    int16_t j4 = 0, j5 = 0, j6 = 0;
    uint8_t has_j4 = 0, has_j5 = 0, has_j6 = 0;
    uint16_t speed = 18000;  // 默认值仅为兼容旧命令，A3不携带速度

    const char *p = strstr((const char *)buf, "J4");
    if (!p) p = strstr((const char *)buf, "j4");
    if (p) { j4 = (int16_t)atoi(p + 2); has_j4 = 1; }

    p = strstr((const char *)buf, "J5");
    if (!p) p = strstr((const char *)buf, "j5");
    if (p) { j5 = (int16_t)atoi(p + 2); has_j5 = 1; }

    p = strstr((const char *)buf, "J6");
    if (!p) p = strstr((const char *)buf, "j6");
    if (p) { j6 = (int16_t)atoi(p + 2); has_j6 = 1; }

    p = strstr((const char *)buf, "S");
    if (!p) p = strstr((const char *)buf, "s");
    if (p) {
        speed = (uint16_t)atoi(p + 1);
        if (speed > 36000) speed = 36000;
        speed = speed * 100;  // 转换到 0.01dps/LSB
    }

    if (!has_j4 && !has_j5 && !has_j6) return;

    if (!has_j5) j5 = (int16_t)(mw_coupled[side].requested_j5_deg1000 / 1000);
    if (!has_j6) j6 = (int16_t)(mw_coupled[side].requested_j6_deg1000 / 1000);

    /* 正解算：关节角度 → 电机目标角度
       统一走 MW_ForwardKin（文本/二进制唯一基准），含 J5×5/3、J6 耦合与范围检查。
       未提供的关节沿用上一次目标（deg×1000 = 1000 计数/度）。 */
    if (side < MW_BUS_COUNT) {
        int64_t j4d = (int64_t)j4 * 1000;
        int64_t j5d = (int64_t)j5 * 1000;
        int64_t j6d = (int64_t)j6 * 1000;
        if (!has_j4) j4d = mw_motors[side][0].target_angle;
        if (!has_j5) j5d = mw_coupled[side].requested_j5_deg1000;
        if (!has_j6) j6d = mw_coupled[side].requested_j6_deg1000;

        int64_t m4, m5, m6;
        if (MW_ForwardKin(side, j4d, j5d, j6d, &m4, &m5, &m6) != 0) {
            int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                             "ERR OUT OF RANGE\r\n");
            Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
            return;
        }

        if (has_j4) {
            mw_motors[side][0].target_angle = m4;
            mw_motors[side][0].target_speed = speed;
            mw_motors[side][0].cmd_pending = 1;
        }

        /* J5/J6作为一组事务；新目标先进入pending，不能覆盖active目标。 */
        if (has_j5 || has_j6) {
            mw_coupled[side].requested_j5_deg1000 = j5d;
            mw_coupled[side].requested_j6_deg1000 = j6d;
            mw_coupled[side].pending_motor5_target = m5;
            mw_coupled[side].pending_motor6_target = m6;
            mw_coupled[side].pending = 1;
        }

        /* 回显 */
        int n = snprintf(g_mw_reply_buf, sizeof(g_mw_reply_buf),
                         "OK %c J4=%d J5=%d J6=%d S=%d\r\n",
                         side == 0 ? 'L' : 'R', j4, j5, j6, speed / 100);
        Host_ReplyBytes((const uint8_t *)g_mw_reply_buf, (uint16_t)n);
    }
}

/* 校验 MW 应答帧（0x88/0x80/0xA3/0x92 共用）：帧头/ID/CMD/和校验/长度。
   供控制序列应答确认与状态机使用；不修改任何状态。 */
uint8_t MW_ReplyOk(uint8_t bus_idx, uint8_t id, uint8_t cmd){
    if (bus_idx >= MW_BUS_COUNT) return 0;
    MW_Bus_t *bus = &mw_buses[bus_idx];
    const uint8_t *rx = bus->rx_buf;
    uint16_t len = bus->rx_len;

    if (len < 5U || len > 64U) return 0;
    if (rx[0] != 0x3E || rx[1] != cmd || rx[2] != id) return 0;
    if (MW_Checksum(rx, 4) != rx[4]) return 0;
    uint8_t data_len = rx[3];
    if (data_len == 0) return 1;                    /* 无数据应答（0x88/0x80） */
    if (data_len > 0U) {
        if ((uint16_t)(5U + data_len + 1U) != len) return 0;
        if (MW_Checksum(rx + 5, data_len) != rx[5 + data_len]) return 0;
    }
    return 1;
}

/* 重置空闲读取事务（STOP/DISABLE/新事务开始时调用） */
void MW_IdlePollReset(uint8_t side)
{
    if (side >= MW_BUS_COUNT) return;
    mw_idle_poll_cnt[side] = MW_IDLE_POLL_TICKS;
    mw_idle_owner[side] = MW_IDLE_NONE;
    mw_idle_next[side] = MW_IDLE_READ_J4;
}

/* 冻结/恢复空闲位置轮询（自检用）：freeze=1 时暂停该总线空闲读，
   防止空闲读干扰重试耗尽等确定性测试。 */
void MW_IdlePollSet(uint8_t side, uint8_t freeze)
{
    if (side >= MW_BUS_COUNT) return;
    if (freeze) {
        mw_idle_poll_cnt[side] = 255;
        mw_idle_owner[side] = MW_IDLE_NONE;
    } else {
        mw_idle_poll_cnt[side] = MW_IDLE_POLL_TICKS;
        mw_idle_owner[side] = MW_IDLE_NONE;
        mw_idle_next[side] = MW_IDLE_READ_J4;
    }
}

/* 文本 L/R STOP 与二进制 STOP 统一走异步全局 STOP 序列（aimotor.c
   Aimotor_ControlStopStart → CTRL_SEQ_STOP），由序列逐台发送 0x80 并回包确认。
   不再提供"连续发送并直接清零 enabled"的单臂停止入口。 */

/* 统一关节→电机换算（文本/二进制唯一基准）。
   输入 deg×1000（1000 计数/度），输出电机侧目标（含右臂取反与 J5/J6 耦合）。
   换算基于已通过文本命令/真机验证的公式（输入已含 ×1000，故仅乘传动比）：
     左臂 M5 = J5×1000 × 5/3；M6 = J6×1000 × 20/9 − M5
     右臂 M5 = −J5×1000 × 5/3
   先乘后除使用 int64，避免精度损失；任何参数超范围返回 1。 */
uint8_t MW_ForwardKin(uint8_t side, int64_t j4_deg1000, int64_t j5_deg1000,
                      int64_t j6_deg1000, int64_t *m4, int64_t *m5, int64_t *m6)
{
    if (side >= MW_BUS_COUNT || m4 == NULL || m5 == NULL || m6 == NULL) {
        return 1;
    }
    if (j4_deg1000 < -MW_DEG1000_LIMIT || j4_deg1000 > MW_DEG1000_LIMIT ||
        j5_deg1000 < -MW_DEG1000_LIMIT || j5_deg1000 > MW_DEG1000_LIMIT ||
        j6_deg1000 < -MW_DEG1000_LIMIT || j6_deg1000 > MW_DEG1000_LIMIT) {
        return 1;
    }

    int64_t m4v = j4_deg1000;                     /* J4 ×1000 */
    int64_t m5v = j5_deg1000 * 5LL / 3LL;         /* J5×1000 × 5/3 */
    if (side == 1) m5v = -m5v;
    int64_t m6v = j6_deg1000 * 20LL / 9LL - m5v;  /* J6×1000 × 20/9 − M5 */

    *m4 = m4v;
    *m5 = m5v;
    *m6 = m6v;
    return 0;
}

/* 将已验证的换算结果写入电机状态：J4 独立目标 + J5/J6 耦合 pending。 */
void MW_ApplyTargets(uint8_t side, int64_t j4_deg1000, int64_t j5_deg1000,
                     int64_t j6_deg1000, int64_t m4, int64_t m5, int64_t m6)
{
    if (side >= MW_BUS_COUNT) return;
    (void)j4_deg1000;   /* J4 无耦合，电机目标 m4 已等于关节目标 */

    mw_motors[side][0].target_angle = m4;
    mw_motors[side][0].cmd_pending = 1;
    mw_motors[side][0].actual_valid = 0;   /* 新目标执行期间反馈视为非最新 */
    mw_motors[side][1].actual_valid = 0;
    mw_motors[side][2].actual_valid = 0;
    mw_coupled[side].requested_j5_deg1000 = j5_deg1000;
    mw_coupled[side].requested_j6_deg1000 = j6_deg1000;
    mw_coupled[side].pending_motor5_target = m5;
    mw_coupled[side].pending_motor6_target = m6;
    mw_coupled[side].pending = 1;
}

/* 二进制 TARGET 的 MW 侧入口：µrad → deg×1000 → 共享换算 → 下发。
   输入为关节侧 µrad；J5 的 5/3 与 J5/J6 耦合仅在 MW_ForwardKin 中应用一次。 */
void MW_BinaryTarget(uint8_t side, const int32_t joint_urad[6])
{
    int64_t m4, m5, m6, j4d, j5d, j6d;
    if (MW_BinaryConvert(side, joint_urad, &m4, &m5, &m6,
                         &j4d, &j5d, &j6d) != 0) return;
    MW_ApplyTargets(side, j4d, j5d, j6d, m4, m5, m6);
}

/* 二进制 TARGET 换算+范围检查：µrad → deg×1000 → 共享换算。
   返回 0=成功；非 0=参数越界/非法。不写入任何电机状态（原子校验用）。 */
uint8_t MW_BinaryConvert(uint8_t side, const int32_t joint_urad[6],
                         int64_t *m4, int64_t *m5, int64_t *m6,
                         int64_t *j4_deg1000, int64_t *j5_deg1000,
                         int64_t *j6_deg1000)
{
    if (side >= MW_BUS_COUNT || joint_urad == NULL ||
        m4 == NULL || m5 == NULL || m6 == NULL ||
        j4_deg1000 == NULL || j5_deg1000 == NULL || j6_deg1000 == NULL) {
        return 1;
    }
    const double urad_to_deg1000 = 180000.0 / M_PI / 1000000.0;
    int64_t j4d = (int64_t)llround((double)joint_urad[3] * urad_to_deg1000);
    int64_t j5d = (int64_t)llround((double)joint_urad[4] * urad_to_deg1000);
    int64_t j6d = (int64_t)llround((double)joint_urad[5] * urad_to_deg1000);
    *j4_deg1000 = j4d;
    *j5_deg1000 = j5d;
    *j6_deg1000 = j6d;
    return MW_ForwardKin(side, j4d, j5d, j6d, m4, m5, m6);
}

/* 关节侧位置反馈（STATE 用）：电机实际角度 → 关节侧 µrad。
   逆换算与 MW_ForwardKin 互为对应关系（输入/输出均为 deg×1000，即
   1000 计数/度）：
     j4_deg1000 = m4
     j5_deg1000 = ±m5×3/5（右臂取反）
     j6_deg1000 = (m6+m5)×9/20
   deg1000 → µrad：×1000×π/180（double，llround）。 */
void MW_ReadbackUrad(uint8_t side, int64_t *j4_urad, int64_t *j5_urad,
                     int64_t *j6_urad)
{
    if (side >= MW_BUS_COUNT || j4_urad == NULL || j5_urad == NULL ||
        j6_urad == NULL) {
        return;
    }
    const double deg1000_to_urad = 1000.0 * M_PI / 180.0;

    int64_t m4 = mw_motors[side][0].actual_angle;
    int64_t m5 = mw_motors[side][1].actual_angle;
    int64_t m6 = mw_motors[side][2].actual_angle;

    int64_t j4d = m4;                                   /* m4 = j4×1000 */
    int64_t j5d = m5 * 3LL / 5LL;                       /* ÷(5/3)，deg×1000 */
    if (side == 1) j5d = -j5d;
    int64_t j6d = (m6 + m5) * 9LL / 20LL;               /* ÷(20/9)，deg×1000 */

    *j4_urad = (int64_t)llround((double)j4d * deg1000_to_urad);
    *j5_urad = (int64_t)llround((double)j5d * deg1000_to_urad);
    *j6_urad = (int64_t)llround((double)j6d * deg1000_to_urad);
}

/* ======================================================================== */
/*                         初始化                                             */
/* ======================================================================== */

void MW_Init(void)
{
    uint8_t b, m;

    /* ── 总线 #0: USART6 (PC6/PC7), PE0 LED (左臂) ────────────── */
    mw_buses[0].huart    = &huart6;
    mw_buses[0].led_port = GPIOE;
    mw_buses[0].led_pin  = GPIO_PIN_0;
    mw_buses[0].rx_ready = 0;
    mw_buses[0].rx_len   = 0;
    mw_buses[0].current_motor = 0;

    /* ── 总线 #1: UART5 (PC12/PD2), PE1 LED (右臂) ────────────── */
    mw_buses[1].huart    = &huart5;
    mw_buses[1].led_port = GPIOE;
    mw_buses[1].led_pin  = GPIO_PIN_1;
    mw_buses[1].rx_ready = 0;
    mw_buses[1].rx_len   = 0;
    mw_buses[1].current_motor = 0;

    /* ── 初始化电机 ─────────────────────────────────────────────── */
    for (b = 0; b < MW_BUS_COUNT; b++) {
        for (m = 0; m < MW_MOTORS_PER; m++) {
            mw_motors[b][m].id            = m + 4;  /* ROS现用ID：J4/J5/J6 */
            mw_motors[b][m].target_angle  = 0;
            mw_motors[b][m].target_speed  = 18000;  // 默认 180°/s
            mw_motors[b][m].actual_angle  = 0;
            mw_motors[b][m].step          = MW_STEP_IDLE;
            mw_motors[b][m].retry_count   = 0;
            mw_motors[b][m].cmd_pending   = 0;
            mw_motors[b][m].actual_valid  = 0;
            mw_motors[b][m].enabled       = 0;
            mw_motors[b][m].fault         = 0;
        }
        memset(&mw_coupled[b], 0, sizeof(mw_coupled[b]));
        mw_coupled[b].step = MW_COUPLED_IDLE;
        MW_IdlePollReset(b);
    }

    /* ── 启动 DMA IDLE 接收 ─────────────────────────────────────── */
    HAL_UARTEx_ReceiveToIdle_DMA(mw_buses[0].huart,
                                  mw_buses[0].rx_buf, 64);
    HAL_UARTEx_ReceiveToIdle_DMA(mw_buses[1].huart,
                                  mw_buses[1].rx_buf, 64);

    /* ── 电机上电后默认处于运行状态，无需 0x88 使能 ───────────────── */
    /* 清除初始化时可能的残留回包标志 */
    for (b = 0; b < MW_BUS_COUNT; b++) {
        mw_buses[b].rx_ready = 0;
        mw_buses[b].rx_len   = 0;
    }
}

/* ======================================================================== */
/*                        DMA IDLE 回调路由                                 */
/* ======================================================================== */

void MW_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < MW_BUS_COUNT; i++) {
        if (mw_buses[i].huart == huart) {
            if (Size > sizeof(mw_buses[i].rx_buf))
                Size = sizeof(mw_buses[i].rx_buf);
            mw_buses[i].rx_len = Size;
            mw_buses[i].rx_ready = 1;
            HAL_UARTEx_ReceiveToIdle_DMA(huart, mw_buses[i].rx_buf, 64);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

void MW_RecoverRx(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < MW_BUS_COUNT; ++i) {
        if (mw_buses[i].huart == huart) {
            HAL_UART_AbortReceive(huart);
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            mw_buses[i].rx_ready = 0;
            mw_buses[i].rx_len = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(huart, mw_buses[i].rx_buf, 64);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

/* ======================================================================== */
/*                        主状态机调度                                     */
/* ======================================================================== */

static uint8_t MW_CoupledFrameOk(MW_Bus_t *bus, uint8_t id,
                                  uint8_t cmd, uint16_t expected_len)
{
    uint8_t *rx = bus->rx_buf;
    if (!bus->rx_ready || bus->rx_len != expected_len) return 0;
    if (rx[0] != 0x3E || rx[1] != cmd || rx[2] != id) return 0;
    if (MW_Checksum(rx, 4) != rx[4]) return 0;
    if (rx[3] == 0) return 1;
    if ((uint16_t)(5U + rx[3] + 1U) != bus->rx_len) return 0;
    return MW_Checksum(rx + 5, rx[3]) == rx[5 + rx[3]];
}

static int64_t MW_ReadI64LE(const uint8_t *data)
{
    uint64_t raw = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        raw |= ((uint64_t)data[i]) << (8U * i);
    }
    return (int64_t)raw;
}

static uint8_t MW_CoupledTimedOut(const MW_CoupledTask_t *task)
{
    return (uint8_t)((HAL_GetTick() - task->last_tx_tick) >=
                     MW_RESPONSE_TIMEOUT_MS);
}

/* J4 独立轴应答超时判据（30ms，与 MW_RESPONSE_TIMEOUT_MS 一致） */
static uint8_t MW_MotorTimedOut(const MW_Motor_t *motor)
{
    return (uint8_t)((HAL_GetTick() - motor->last_tx_tick) >=
                     MW_RESPONSE_TIMEOUT_MS);
}

static void MW_CoupledProcess(uint8_t bus_idx)
{
    MW_Bus_t *bus = &mw_buses[bus_idx];
    MW_CoupledTask_t *task = &mw_coupled[bus_idx];

    if (task->step == MW_COUPLED_IDLE && task->pending) {
        task->motor5_target = task->pending_motor5_target;
        task->motor6_target = task->pending_motor6_target;
        task->pending = 0;
        task->retry_count = 0;
        task->arrived = 0;
        task->arrive_count = 0;
        task->fault = 0;
        task->move_start_tick = HAL_GetTick();
        task->step = MW_COUPLED_WRITE_M5;
    }

    switch (task->step) {
    case MW_COUPLED_WRITE_M5:
        bus->rx_ready = 0;
        MW_SendA3(bus_idx, 5, task->motor5_target);
        task->last_tx_tick = HAL_GetTick();
        /* 重试计数只在事务启动时清零；重发时保留，确保达到上限后停止 */
        task->step = MW_COUPLED_WAIT_WRITE_M5;
        break;
    case MW_COUPLED_WAIT_WRITE_M5:
        if (MW_CoupledFrameOk(bus, 5, 0xA3, 13)) {
            bus->rx_ready = 0;
            task->step = MW_COUPLED_WRITE_M6;
            task->retry_count = 0;
        } else if (MW_CoupledTimedOut(task)) {
            bus->rx_ready = 0;
            if (++task->retry_count > MW_RETRY_MAX) {
                task->fault = 1;
                mw_motors[bus_idx][1].fault = 1;   /* 耦合事务失败 → J5 轴故障 */
                mw_motors[bus_idx][2].fault = 1;   /* 未完成的 M6 侧同样标记 */
                task->step = MW_COUPLED_IDLE;
            } else {
                task->step = MW_COUPLED_WRITE_M5;
            }
        }
        break;
    case MW_COUPLED_WRITE_M6:
        bus->rx_ready = 0;
        MW_SendA3(bus_idx, 6, task->motor6_target);
        task->last_tx_tick = HAL_GetTick();
        task->step = MW_COUPLED_WAIT_WRITE_M6;
        break;
    case MW_COUPLED_WAIT_WRITE_M6:
        if (MW_CoupledFrameOk(bus, 6, 0xA3, 13)) {
            bus->rx_ready = 0;
            task->step = MW_COUPLED_READ_M5;
            task->retry_count = 0;
        } else if (MW_CoupledTimedOut(task)) {
            bus->rx_ready = 0;
            if (++task->retry_count > MW_RETRY_MAX) {
                task->fault = 1;
                mw_motors[bus_idx][1].fault = 1;
                mw_motors[bus_idx][2].fault = 1;
                task->step = MW_COUPLED_IDLE;
            } else {
                task->step = MW_COUPLED_WRITE_M6;
            }
        }
        break;
    case MW_COUPLED_READ_M5:
        bus->rx_ready = 0;
        MW_Send92(bus_idx, 5);
        task->last_tx_tick = HAL_GetTick();
        task->step = MW_COUPLED_WAIT_M5;
        break;
    case MW_COUPLED_WAIT_M5:
        if (MW_CoupledFrameOk(bus, 5, 0x92, 14)) {
            task->motor5_actual = MW_ReadI64LE(&bus->rx_buf[5]);
            mw_motors[bus_idx][1].actual_angle = task->motor5_actual;
            mw_motors[bus_idx][1].actual_valid = 1;
            bus->rx_ready = 0;
            task->step = MW_COUPLED_READ_M6;
            task->retry_count = 0;
        } else if (MW_CoupledTimedOut(task)) {
            bus->rx_ready = 0;
            if (++task->retry_count > MW_RETRY_MAX) {
                task->fault = 1;
                mw_motors[bus_idx][1].fault = 1;
                mw_motors[bus_idx][2].fault = 1;
                task->step = MW_COUPLED_IDLE;
            } else {
                task->step = MW_COUPLED_READ_M5;
            }
        }
        break;
    case MW_COUPLED_READ_M6:
        bus->rx_ready = 0;
        MW_Send92(bus_idx, 6);
        task->last_tx_tick = HAL_GetTick();
        task->step = MW_COUPLED_WAIT_M6;
        break;
    case MW_COUPLED_WAIT_M6:
        if (MW_CoupledFrameOk(bus, 6, 0x92, 14)) {
            task->motor6_actual = MW_ReadI64LE(&bus->rx_buf[5]);
            mw_motors[bus_idx][2].actual_angle = task->motor6_actual;
            mw_motors[bus_idx][2].actual_valid = 1;
            bus->rx_ready = 0;
#if MW_ENABLE_ARRIVAL_CHECK
            int64_t e5 = task->motor5_actual - task->motor5_target;
            int64_t e6 = (task->motor6_actual + task->motor5_actual) -
                         (task->motor6_target + task->motor5_target);
            if (e5 < 0) e5 = -e5;
            if (e6 < 0) e6 = -e6;
            if (e5 <= MW_ARRIVAL_TOLERANCE &&
                e6 <= MW_ARRIVAL_TOLERANCE) {
                if (task->arrive_count < MW_ARRIVE_SAMPLES)
                    task->arrive_count++;
            } else {
                task->arrive_count = 0;
            }
            task->arrived = (task->arrive_count >= MW_ARRIVE_SAMPLES);
            task->retry_count = 0;
            if (task->pending) {
                task->step = MW_COUPLED_IDLE;
            } else if (task->arrived ||
                       (HAL_GetTick() - task->move_start_tick) >=
                           MW_MOVE_TIMEOUT_MS) {
                if (!task->arrived) task->fault = 1;
                task->step = MW_COUPLED_IDLE;
            } else {
                task->step = MW_COUPLED_READ_M5;
            }
#else
            /* 到位检测暂时注释：先完成一轮反馈，再回到空闲等待下一条命令。 */
            task->arrived = 0;
            task->arrive_count = 0;
            task->fault = 0;
            mw_motors[bus_idx][1].fault = 0;   /* 耦合事务成功完成，清除 J5/J6 故障 */
            mw_motors[bus_idx][2].fault = 0;
            task->step = MW_COUPLED_IDLE;
#endif
        } else if (MW_CoupledTimedOut(task)) {
            bus->rx_ready = 0;
            if (++task->retry_count > MW_RETRY_MAX) {
                task->fault = 1;
                mw_motors[bus_idx][1].fault = 1;
                mw_motors[bus_idx][2].fault = 1;
                task->step = MW_COUPLED_IDLE;
            } else {
                task->step = MW_COUPLED_READ_M6;
            }
        }
        break;
    default:
        break;
    }
}

void MW_Process(void)
{
    uint8_t b;

    /* 文本命令不刷新 ROS 2 通信看门狗（仅二进制有效命令刷新）。
       控制序列进行中仅允许 L/R STOP 穿透并抢占；其它文本动作继续排队，
       避免与序列在总线上冲突。 */
    if (g_mw_cmd_ready) {
        if (g_mw_cmd_buf[0] == 'L' || g_mw_cmd_buf[0] == 'l' ||
            g_mw_cmd_buf[0] == 'R' || g_mw_cmd_buf[0] == 'r') {
            uint8_t is_stop = (strstr((const char *)g_mw_cmd_buf, "STOP") != NULL ||
                               strstr((const char *)g_mw_cmd_buf, "stop") != NULL);
            if (!Aimotor_CtrlSeqActive() || is_stop) {
                g_mw_cmd_ready = 0;
                MW_CmdParse();
            }
        }
    }

    /* 控制序列进行中：暂停 MW 状态机与空闲轮询，序列独占总线 */
    if (Aimotor_CtrlSeqActive()) {
        return;
    }

    for (b = 0; b < MW_BUS_COUNT; b++) {
        MW_Motor_t *motor = &mw_motors[b][0];
        uint8_t coupled_busy = (mw_coupled[b].step != MW_COUPLED_IDLE) ||
                               mw_coupled[b].pending;

        /* 仅在整条总线空闲时推进公共倒计时；J4 与 J5->J6 整轮交替，
           避免 J5/J6 总在前半段抢占倒计时而饿死 J4。 */
        if (!coupled_busy &&
            motor->step == MW_STEP_IDLE &&
            !motor->cmd_pending &&
            mw_idle_owner[b] == MW_IDLE_NONE) {
            if (mw_idle_poll_cnt[b] > 0) {
                mw_idle_poll_cnt[b]--;
            } else {
                mw_idle_owner[b] = mw_idle_next[b];
            }
        }

        /* 耦合路径：正常命令事务或空闲耦合读。回包只由耦合逻辑（ID5/6）消费，
           owner 为 J5/J6 时任何 J4 应答不会被 J4 路径消费。 */
        if (coupled_busy ||
            mw_idle_owner[b] == MW_IDLE_READ_J5 ||
            mw_idle_owner[b] == MW_IDLE_READ_J6) {
            if (mw_coupled[b].step == MW_COUPLED_IDLE &&
                !mw_coupled[b].pending &&
                mw_idle_owner[b] == MW_IDLE_READ_J5 &&
                !mw_coupled[b].fault) {
                mw_coupled[b].step = MW_COUPLED_READ_M5;   /* 启动空闲耦合读 */
            }
            MW_CoupledProcess(b);
            if (mw_coupled[b].step == MW_COUPLED_IDLE &&
                !mw_coupled[b].pending &&
                (mw_idle_owner[b] == MW_IDLE_READ_J5 ||
                 mw_idle_owner[b] == MW_IDLE_READ_J6)) {
                /* 耦合读完成（成功或超时 fault 后均回到 IDLE）：释放 owner */
                mw_idle_owner[b] = MW_IDLE_NONE;
                mw_idle_next[b] = MW_IDLE_READ_J4;
                mw_idle_poll_cnt[b] = MW_IDLE_POLL_TICKS;
                mw_coupled[b].fault = 0;
            }
            continue;
        }

        /* J4独立轴，使用ID=4。 */
        if (motor->step == MW_STEP_IDLE && motor->cmd_pending) {
            motor->cmd_pending = 0;
            motor->step = MW_STEP_WRITE;
            motor->retry_count = 0;   /* 新事务启动时清零重试 */
        }

        /* 调度器选中 J4 后启动一次 0x92；完成或重试耗尽回到 IDLE 后，
           下一轮切换到 J5->J6。 */
        if (motor->step == MW_STEP_IDLE &&
            mw_idle_owner[b] == MW_IDLE_READ_J4 &&
            mw_idle_next[b] == MW_IDLE_READ_J4) {
            motor->step = MW_STEP_QUERY;
            motor->retry_count = 0;
            mw_idle_next[b] = MW_IDLE_READ_J5;
        } else if (motor->step == MW_STEP_IDLE &&
                   mw_idle_owner[b] == MW_IDLE_READ_J4) {
            mw_idle_owner[b] = MW_IDLE_NONE;
            mw_idle_poll_cnt[b] = MW_IDLE_POLL_TICKS;
        }

        /* J4 应答只由 J4（ID=4）路径消费；其它 ID 的应答不在此清除，
           避免空闲读取之间相互消费错误回包。 */
        if (mw_buses[b].rx_ready && motor->step == MW_STEP_WAIT_RX &&
            mw_buses[b].rx_buf[2] == motor->id) {
            ProcessRx(b, 0);   /* 成功后置 retry_count=0、step=IDLE */
        }
        if (motor->step == MW_STEP_WAIT_WRITE &&
            MW_CoupledFrameOk(&mw_buses[b], motor->id, 0xA3, 13)) {
            mw_buses[b].rx_ready = 0;
            motor->retry_count = 0;
            motor->step = MW_STEP_QUERY;
        }

        switch (motor->step) {
        case MW_STEP_WRITE:
            mw_buses[b].rx_ready = 0;
            MW_SendA3(b, motor->id, motor->target_angle);
            motor->last_tx_tick = HAL_GetTick();
            /* 重试计数保留，重发时递增；仅新事务/成功时清零 */
            motor->step = MW_STEP_WAIT_WRITE;
            break;
        case MW_STEP_WAIT_WRITE:
            if (MW_MotorTimedOut(motor)) {
                mw_buses[b].rx_ready = 0;
                if (++motor->retry_count > MW_RETRY_MAX) {
                    motor->retry_count = 0;
                    motor->fault = 1;
                    motor->step = MW_STEP_IDLE;   /* 达到上限：结束事务 */
                } else {
                    motor->step = MW_STEP_WRITE;  /* 重发 */
                }
            }
            break;
        case MW_STEP_QUERY:
            mw_buses[b].rx_ready = 0;
            MW_Send92(b, motor->id);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MW_STEP_WAIT_RX;
            break;
        case MW_STEP_WAIT_RX:
            if (MW_MotorTimedOut(motor)) {
                mw_buses[b].rx_ready = 0;
                if (++motor->retry_count > MW_RETRY_MAX) {
                    motor->retry_count = 0;
                    motor->fault = 1;
                    motor->step = MW_STEP_IDLE;   /* 达到上限：结束事务 */
                } else {
                    motor->step = MW_STEP_QUERY;  /* 重发 */
                }
            }
            break;
        default:
            break;
        }

    }
}

/* USER CODE END 0 */
