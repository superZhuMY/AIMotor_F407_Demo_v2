/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : host_protocol.c
  * @brief          : 上位机协议层：控制状态机/通信看门狗/流解析/二进制帧/文本命令
  *                   （自 aimotor.c 与 main.c 拆分，逻辑逐行搬迁未改动）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "aimotor.h"
#include "aimotor_internal.h"
#include "host_protocol.h"
#include "mwmotor.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>

/* USER CODE BEGIN 0 */

/* ======================================================================== */
/*                上位机流接收环形缓冲（拆包/粘包/CRC 重同步）                  */
/* ======================================================================== */

static uint8_t  g_host_rx_ring[PROTOCOL_RX_BUFFER_SIZE];
volatile uint16_t g_host_ring_head = 0;  /* 下一个写入位置 */
volatile uint16_t g_host_ring_tail = 0;  /* 下一个读取位置 */

/* 文本行提取缓冲（流解析 → 现有命令缓冲） */
static char     g_host_text_buf[64];
uint16_t g_host_text_len = 0;
static uint32_t g_host_text_activity_tick = 0;

/* ======================================================================== */
/*                      安全层：控制状态机 + 通信看门狗                        */
/* ======================================================================== */

ControlState_t g_control_state = CONTROL_DISABLED;
uint8_t        g_control_faults = CONTROL_FAULT_NONE;
uint32_t       g_last_valid_frame_ms = 0;  /* 上次有效命令时间 */

/* ── 上位机命令槽（原 main.c 定义随协议层迁入；旧别名 g_host_rx_* 更名 g_mw_cmd_*） ── */
char              g_host_cmd_buf[64];   /* 命令缓冲（流解析填充，Aimotor_Process 消费） */
volatile uint16_t g_host_cmd_len = 0;
volatile uint8_t  g_host_cmd_ready = 0;

uint8_t           g_mw_cmd_buf[64];     /* L/R 命令缓冲（流解析填充，MW_Process 消费） */
volatile uint16_t g_mw_cmd_len = 0;
volatile uint8_t  g_mw_cmd_ready = 0;

/* USART1 DMA 接收暂存（comm_router.c 回调写入） */
uint8_t g_host_dma_buf[64];

/* 文本命令错误/确认回复缓冲（HostCmd_Parse/Execute 专用） */
static char g_host_tx_buf[96];

/* USART1 异步（IT）发送的持久 TX 缓冲：HAL_UART_Transmit_IT 只保存源地址、
   不做拷贝，调用方若传入局部栈数组，函数返回后异步发送期间数据即失效。
   所有回复先拷入此处再启动 IT 发送。80 字节覆盖当前最大 STATE 帧 70 字节。 */
#define HOST_TX_BUF_SIZE 80U
static uint8_t g_host_tx_async_buf[HOST_TX_BUF_SIZE];

static int32_t BinaryReadI32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

/* ACK 帧：AA 55 | VER | 0x80 | SEQ(2) | LEN=4 | seq(2)+cmd(1)+result(1) | CRC(2)
   LEN 与实际 payload(4) 严格一致，总长 10+4=14，CRC 位于 [12..13]。 */
void BinarySendAck(uint16_t seq, uint8_t cmd, uint8_t result)
{
    uint8_t out[14];
    out[0] = AIMOTOR_BINARY_SOF0;
    out[1] = AIMOTOR_BINARY_SOF1;
    out[2] = AIMOTOR_BINARY_VERSION;
    out[3] = AIMOTOR_BINARY_RESP_ACK;
    out[4] = (uint8_t)(seq & 0xFF);
    out[5] = (uint8_t)(seq >> 8);
    out[6] = 4;                       /* LEN = 4 */
    out[7] = 0;
    out[8] = (uint8_t)(seq & 0xFF);   /* payload[0..1]: 请求 SEQ */
    out[9] = (uint8_t)(seq >> 8);
    out[10] = cmd;                    /* payload[2]: 回显命令码 */
    out[11] = result;                 /* payload[3]: 结果码 */
    uint16_t crc = Aimotor_CRC16(&out[2], 10);   /* VER..result */
    out[12] = (uint8_t)(crc & 0xFF);
    out[13] = (uint8_t)(crc >> 8);
#if AIMOTOR_SELF_TEST
    if (g_test_capture_tx) {
        uint16_t n = (g_test_tx_len + 14U <= sizeof(g_test_tx_log)) ? 14U :
                     (uint16_t)(sizeof(g_test_tx_log) - g_test_tx_len);
        for (uint16_t i = 0; i < n; i++) g_test_tx_log[g_test_tx_len + i] = out[i];
        g_test_tx_len += n;
        return;
    }
#endif
    Host_ReplyBytes(out, sizeof(out));
}

/* 关节位置 → 关节侧单位逆换算辅助（µrad） */
static int32_t ClampI32(int64_t v)
{
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}

/* AI 脉冲 → 关节侧 µm（与二进制 TARGET 的 µm→脉冲系数互为逆运算） */
static int32_t AiPulseToUm(int32_t pulse, uint8_t joint)
{
    int64_t v;
    switch (joint) {
    case 0:  v = ((int64_t)pulse * 4LL) / 10LL;      break; /* ×10/4 的逆 */
    case 1:  v = ((int64_t)pulse * 475LL) / 10000LL; break; /* ×10000/475 的逆 */
    default: v = ((int64_t)pulse * 2LL) / 10LL;      break; /* ×10/2 的逆 */
    }
    return ClampI32(v);
}

/* STATE 帧：AA 55 | VER | 0x81 | SEQ(2)=请求seq | LEN=60 |
   payload: seq(2)+state(1)+fault(1)+valid(2)+enabled(2)+axis_status(4)+12×int32
   总长 10+60=70。轴序固定：L_J1..L_J6, R_J1..R_J6，所有位图统一 side*6+joint。
   valid:      bit=该轴有真实回读
   enabled:    bit=该轴已确认使能（ENABLE 序列 0x88/ServoOn 回包确认后置位）
   axis_status:uint32，每轴 2bit（bit0=online 真实回读，bit1=fault）——共 24bit，
               右臂 J3..J6（bit16..22）不再丢失。
   actual_valid==0 的轴位置序列化为 0，绝不发送缓存旧位置。 */
static void Aimotor_SendState(uint16_t seq)
{
    uint8_t out[70];
    uint16_t n = 0;

    out[n++] = AIMOTOR_BINARY_SOF0;
    out[n++] = AIMOTOR_BINARY_SOF1;
    out[n++] = AIMOTOR_BINARY_VERSION;
    out[n++] = AIMOTOR_BINARY_RESP_STATE;
    out[n++] = (uint8_t)(seq & 0xFF);
    out[n++] = (uint8_t)(seq >> 8);
    out[n++] = 60;                    /* LEN */
    out[n++] = 0;
    /* payload 起点 [8] */
    out[n++] = (uint8_t)(seq & 0xFF);
    out[n++] = (uint8_t)(seq >> 8);
    out[n++] = (uint8_t)g_control_state;
    out[n++] = g_control_faults;
    uint16_t valid = 0;
    uint16_t enabled_bits = 0;
    uint32_t axis_status = 0;   /* 每轴 2bit：bit0=online(真实回读)，bit1=fault */
    for (uint8_t side = 0; side < AIMOTOR_BUS_COUNT; side++) {
        for (uint8_t i = 0; i < AIMOTOR_MOTORS_PER; i++) {
            uint16_t bit = AIMOTOR_AXIS_BIT(side, i);
            if (aimotor_motors[side][i].actual_valid) {
                valid |= bit;
                axis_status |= (uint32_t)(1UL << (side * 12U + i * 2U));
            }
            if (aimotor_motors[side][i].enabled) enabled_bits |= bit;
            if (aimotor_motors[side][i].fault)
                axis_status |= (uint32_t)(2UL << (side * 12U + i * 2U));
        }
        for (uint8_t i = 0; i < MW_MOTORS_PER; i++) {
            uint16_t bit = AIMOTOR_AXIS_BIT(side, 3U + i);
            if (mw_motors[side][i].actual_valid) {
                valid |= bit;
                axis_status |= (uint32_t)(1UL << (side * 12U + (3U + i) * 2U));
            }
            if (mw_motors[side][i].enabled) enabled_bits |= bit;
            if (mw_motors[side][i].fault)
                axis_status |= (uint32_t)(2UL << (side * 12U + (3U + i) * 2U));
        }
    }
    out[n++] = (uint8_t)(valid & 0xFF);
    out[n++] = (uint8_t)(valid >> 8);
    out[n++] = (uint8_t)(enabled_bits & 0xFF);
    out[n++] = (uint8_t)(enabled_bits >> 8);
    out[n++] = (uint8_t)(axis_status & 0xFF);
    out[n++] = (uint8_t)((axis_status >> 8) & 0xFF);
    out[n++] = (uint8_t)((axis_status >> 16) & 0xFF);
    out[n++] = (uint8_t)((axis_status >> 24) & 0xFF);

    /* 12 轴关节侧位置，LE int32；无效轴填 0 */
    for (uint8_t side = 0; side < AIMOTOR_BUS_COUNT; side++) {
        for (uint8_t i = 0; i < AIMOTOR_MOTORS_PER; i++) {
            int32_t v = 0;
            if (aimotor_motors[side][i].actual_valid) {
                v = AiPulseToUm(aimotor_motors[side][i].actual_position, i);
            }
            out[n++] = (uint8_t)(v & 0xFF);
            out[n++] = (uint8_t)((v >> 8) & 0xFF);
            out[n++] = (uint8_t)((v >> 16) & 0xFF);
            out[n++] = (uint8_t)((v >> 24) & 0xFF);
        }
        /* J4..J6 (MW, µrad) */
        int64_t j4 = 0, j5 = 0, j6 = 0;
        MW_ReadbackUrad(side, &j4, &j5, &j6);
        int32_t vals[3] = { 0, 0, 0 };
        if (mw_motors[side][0].actual_valid) vals[0] = ClampI32(j4);
        if (mw_motors[side][1].actual_valid) vals[1] = ClampI32(j5);
        if (mw_motors[side][2].actual_valid) vals[2] = ClampI32(j6);
        for (uint8_t i = 0; i < 3; i++) {
            int32_t v = vals[i];
            out[n++] = (uint8_t)(v & 0xFF);
            out[n++] = (uint8_t)((v >> 8) & 0xFF);
            out[n++] = (uint8_t)((v >> 16) & 0xFF);
            out[n++] = (uint8_t)((v >> 24) & 0xFF);
        }
    }

    uint16_t crc = Aimotor_CRC16(&out[2], (uint16_t)(6U + 60U));
    out[n++] = (uint8_t)(crc & 0xFF);
    out[n++] = (uint8_t)(crc >> 8);
#if AIMOTOR_SELF_TEST
    if (g_test_capture_tx) {
        uint16_t cap = (g_test_tx_len + n <= sizeof(g_test_tx_log)) ? n :
                       (uint16_t)(sizeof(g_test_tx_log) - g_test_tx_len);
        for (uint16_t i = 0; i < cap; i++) g_test_tx_log[g_test_tx_len + i] = out[i];
        g_test_tx_len += cap;
        return;
    }
#endif
    Host_ReplyBytes(out, n);
}

/* ======================================================================== */
/*               上位机二进制协议：流解析 / 状态机 / 命令分发                     */
/* ======================================================================== */

/* TARGET 整帧原子校验+下发：
   Phase 1 只校验（AI 脉冲范围 + MW 换算范围），任何错误返回非 0、不执行任何轴；
   Phase 2 全部合法才统一下发。TARGET 仅在 CONTROL_ENABLED 状态允许（分发层检查）。 */
static uint8_t Aimotor_ArmHasLatchedFault(uint8_t side)
{
    if (side >= AIMOTOR_BUS_COUNT) return 1;
    for (uint8_t i = 0; i < AIMOTOR_MOTORS_PER; i++) {
        if (aimotor_motors[side][i].fault) return 1;
    }
    for (uint8_t i = 0; i < MW_MOTORS_PER; i++) {
        if (mw_motors[side][i].fault) return 1;
    }
    return mw_coupled[side].fault ? 1U : 0U;
}

static uint8_t Aimotor_AllAxesConfirmedEnabled(void)
{
    for (uint8_t side = 0; side < AIMOTOR_BUS_COUNT; side++) {
        if (Aimotor_ArmHasLatchedFault(side)) return 0;
        for (uint8_t i = 0; i < AIMOTOR_MOTORS_PER; i++) {
            if (!aimotor_motors[side][i].enabled) return 0;
        }
        for (uint8_t i = 0; i < MW_MOTORS_PER; i++) {
            if (!mw_motors[side][i].enabled) return 0;
        }
    }
    return 1;
}

static uint8_t BinaryTargetValidateAndDispatch(uint8_t side,
                                               const int32_t joints[6])
{
    int64_t ai_pulse[3];

    /* 运动通信故障锁存到下一次成功 ENABLE。禁止新 TARGET 通过清除 fault
       自动重启事务；上位机应先 STOP/DISABLE，再显式 ENABLE 恢复。 */
    if (Aimotor_ArmHasLatchedFault(side)) {
        return AIMOTOR_ACK_CTRL_FAILED;
    }

    for (uint8_t i = 0; i < 3; i++) {
        int64_t um = joints[i];
        int64_t pulse;
        if (i == 0) pulse = (um * 10LL) / 4LL;
        else if (i == 1) pulse = (um * 10000LL) / 475LL;
        else pulse = (um * 10LL) / 2LL;
        if (pulse < AIMOTOR_POSITION_MIN || pulse > AIMOTOR_POSITION_MAX) {
            return AIMOTOR_ACK_OUT_OF_RANGE;
        }
        ai_pulse[i] = pulse;
    }

    int64_t m4, m5, m6, j4d, j5d, j6d;
    if (MW_BinaryConvert(side, joints, &m4, &m5, &m6,
                         &j4d, &j5d, &j6d) != 0) {
        return AIMOTOR_ACK_OUT_OF_RANGE;
    }

    /* Phase 2：全部合法，统一下发。
       TARGET 是整臂帧，但 ROS 轨迹控制会重复发送保持位置；只把真正变化
       的 AI 轴标为待执行，避免某一腕部关节运动时重新触发三个平移轴的
       STOP → WRITE → TRIGGER 事务。 */
    for (uint8_t i = 0; i < 3; i++) {
        if (aimotor_motors[side][i].target_position != (int32_t)ai_pulse[i]) {
            aimotor_motors[side][i].target_position = (int32_t)ai_pulse[i];
            aimotor_motors[side][i].cmd_pending = 1;
        }
    }
    MW_ApplyTargets(side, j4d, j5d, j6d, m4, m5, m6);
    return AIMOTOR_ACK_OK;
}

/* 帧长度/版本/CRC 已校验通过后按 CMD 分发。
   返回分发结果：仅 FRAME_ACCEPTED_REFRESH_WATCHDOG 的命令可刷新通信看门狗。
   刷新表（最终定义，见协议文档 §6.6）：
     ENABLE/TARGET/GET_STATE/STOP/DISABLE 成功接受 → 刷新；
     HELLO 成功接受 → 固定不刷新；
     长度错/未知CMD/arm错/越界/状态拒绝/序列忙/控制失败 → 不刷新。 */
static FrameDispatchResult_t Aimotor_HandleBinaryFrame(const uint8_t *f,
                                                       uint16_t len,
                                                       uint16_t total_len)
{
    uint16_t seq = (uint16_t)f[4] | ((uint16_t)f[5] << 8);
    uint8_t cmd = f[3];

    (void)total_len;

    switch (cmd) {
    case AIMOTOR_BINARY_CMD_HELLO:
        if (len != 0U) { BinarySendAck(seq, cmd, AIMOTOR_ACK_BAD_LEN); return FRAME_REJECTED; }
        BinarySendAck(seq, cmd, AIMOTOR_ACK_OK);
        return FRAME_ACCEPTED_NO_WATCHDOG;   /* HELLO 固定不刷新 */

    case AIMOTOR_BINARY_CMD_ENABLE:
        if (len != 0U) { BinarySendAck(seq, cmd, AIMOTOR_ACK_BAD_LEN); return FRAME_REJECTED; }
        if (Aimotor_CtrlSeqActive()) {
            BinarySendAck(seq, cmd, AIMOTOR_ACK_CTRL_BUSY);
            return FRAME_REJECTED;
        }
        if ((g_control_faults & CONTROL_FAULT_INTERNAL) != 0U) {
            /* 不可恢复故障（内部状态损坏）：拒绝 ENABLE，只能复位恢复 */
            BinarySendAck(seq, cmd, AIMOTOR_ACK_CTRL_FAILED);
            return FRAME_REJECTED;
        }
        if (g_control_state == CONTROL_ENABLED) {
            /* 幂等 ENABLE：已经完整确认使能时只回 ACK 并刷新看门狗，绝不
               重跑最长数秒的 12 轴序列，避免序列活动屏蔽 250ms 看门狗。
               若软件状态与轴状态不一致，则拒绝并要求先安全停止再恢复。 */
            if (!Aimotor_AllAxesConfirmedEnabled()) {
                BinarySendAck(seq, cmd, AIMOTOR_ACK_CTRL_FAILED);
                return FRAME_REJECTED;
            }
            BinarySendAck(seq, cmd, AIMOTOR_ACK_OK);
            return FRAME_ACCEPTED_REFRESH_WATCHDOG;
        }
        CtrlSeqStart(CTRL_SEQ_ENABLE, seq, cmd, 1);
        /* 看门狗刷新延迟到序列成功完成时（见 CtrlSeqFinish），
           失败（0x0A）不刷新。 */
        return FRAME_ACCEPTED_NO_WATCHDOG;

    case AIMOTOR_BINARY_CMD_STOP:
        /* 全局安全停止双臂 12 轴；arm 字段仅作兼容保留并明确忽略 */
        if (len != 1U) { BinarySendAck(seq, cmd, AIMOTOR_ACK_BAD_LEN); return FRAME_REJECTED; }
        CtrlSeqStart(CTRL_SEQ_STOP, seq, cmd, 1);   /* STOP 最高优先级，可抢占 */
        return FRAME_ACCEPTED_NO_WATCHDOG;

    case AIMOTOR_BINARY_CMD_GET_STATE:
        if (len != 0U) { BinarySendAck(seq, cmd, AIMOTOR_ACK_BAD_LEN); return FRAME_REJECTED; }
        Aimotor_SendState(seq);
        return FRAME_ACCEPTED_REFRESH_WATCHDOG;

    case AIMOTOR_BINARY_CMD_DISABLE:
        if (len != 0U) { BinarySendAck(seq, cmd, AIMOTOR_ACK_BAD_LEN); return FRAME_REJECTED; }
        if (Aimotor_CtrlSeqActive()) {
            BinarySendAck(seq, cmd, AIMOTOR_ACK_CTRL_BUSY);
            return FRAME_REJECTED;
        }
        CtrlSeqStart(CTRL_SEQ_DISABLE, seq, cmd, 1);
        return FRAME_ACCEPTED_NO_WATCHDOG;

    case AIMOTOR_BINARY_CMD_TARGET: {
        if (g_control_state != CONTROL_ENABLED) {
            BinarySendAck(seq, cmd, AIMOTOR_ACK_STATE_DENIED);
            return FRAME_REJECTED;
        }
        if (Aimotor_CtrlSeqActive()) {
            /* 控制序列占用总线时拒绝 TARGET，避免与序列事务冲突 */
            BinarySendAck(seq, cmd, AIMOTOR_ACK_CTRL_BUSY);
            return FRAME_REJECTED;
        }
        if (len != 28U) { BinarySendAck(seq, cmd, AIMOTOR_ACK_BAD_LEN); return FRAME_REJECTED; }
        uint8_t side = f[8];
        if (side > 1U) { BinarySendAck(seq, cmd, AIMOTOR_ACK_BAD_ARM); return FRAME_REJECTED; }
        /* 关节从 payload 偏移 4（f[12]）开始：arm/mode/flags 共 4 字节 */
        const uint8_t *jp = &f[12];
        int32_t joints[6];
        for (uint8_t i = 0; i < 6; i++) joints[i] = BinaryReadI32(&jp[4U * i]);
        uint8_t res = BinaryTargetValidateAndDispatch(side, joints);
        BinarySendAck(seq, cmd, res);
        if (res == AIMOTOR_ACK_OK) {
            return FRAME_ACCEPTED_REFRESH_WATCHDOG;   /* 已接受，刷新 */
        }
        return FRAME_REJECTED;   /* 越界/其他错误：不刷新 */
    }

    default:
        BinarySendAck(seq, cmd, AIMOTOR_ACK_UNKNOWN_CMD);
        return FRAME_REJECTED;   /* 未知 CMD：不刷新 */
    }
}


/* DMA IDLE 回调调用：只把收到的字节追加入环形缓冲，ISR 内不做解析。 */
void Aimotor_HostRxAppend(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((g_host_ring_head + 1U) %
                                   PROTOCOL_RX_BUFFER_SIZE);
        if (next == g_host_ring_tail) {
            /* 缓冲满：标记溢出（流解析时上报），不覆盖未解析数据 */
            g_control_faults |= CONTROL_FAULT_RX_OVERFLOW;
            return;
        }
        g_host_rx_ring[g_host_ring_head] = data[i];
        g_host_ring_head = next;
    }
}

/* 流解析：拆包/粘包/CRC 重同步。
   环形缓冲只保存字节流；本函数在主循环中解析：
   1) 查找连续的 AA 55；2) 帧头前的字节按噪声/文本丢弃；
   3) 缓存不足 8 字节时等待下一批数据；4) 读 LEN，超过最大 payload 则
   跳到下一个可能的帧头并记录溢出；5) 计算 total=10+LEN，不足则等待；
   6) 校验 VERSION/LEN/CRC；7) 校验成功分发命令；8) 消费该帧；
   9) 继续解析剩余数据；10) CRC 错误重同步，不卡死在错误帧上。
   非 AA 字节按文本行聚合并冲刷到现有命令缓冲（\n 或 50ms 超时）。
   栈上仅使用一个 256B 的帧拷贝缓冲。 */
void Aimotor_HostStreamPoll(void)
{
    uint32_t now = HAL_GetTick();
    /* 解析进度第二层保护：每次调用最多处理 2×缓冲大小 个循环迭代。
       每个迭代要么消费字节、要么成功消费一帧、要么 return，
       循环上限只在极端情况下兜底，不掩盖进度问题。 */
    uint32_t loop_guard = PROTOCOL_RX_BUFFER_SIZE * 2U;

    while (g_host_ring_head != g_host_ring_tail) {
        if (loop_guard-- == 0U) {
            return;   /* 兜底：强制返回，保证主循环与看门狗继续运行 */
        }
        uint8_t b0 = g_host_rx_ring[g_host_ring_tail];

        if (b0 == AIMOTOR_BINARY_SOF0) {
            uint16_t nxt = (uint16_t)((g_host_ring_tail + 1U) %
                                      PROTOCOL_RX_BUFFER_SIZE);
            if (g_host_ring_head == nxt) {
                return;   /* 只有 AA 一个字节，等待后续 */
            }
            uint8_t b1 = g_host_rx_ring[nxt];
            if (b1 == AIMOTOR_BINARY_SOF1) {
                /* 确认二进制帧头 AA 55 */
                uint16_t avail;
                if (g_host_ring_head >= g_host_ring_tail) {
                    avail = (uint16_t)(g_host_ring_head - g_host_ring_tail);
                } else {
                    /* 环形缓冲已回绕：显式 + SIZE 避免无符号取模错误 */
                    avail = (uint16_t)((g_host_ring_head +
                                        PROTOCOL_RX_BUFFER_SIZE) -
                                       g_host_ring_tail);
                }
                if (avail < AIMOTOR_BINARY_HEADER_SIZE) {
                    return;   /* 不足 8 字节，等待 */
                }
                uint16_t len = (uint16_t)g_host_rx_ring[
                                   (g_host_ring_tail + 6U) %
                                   PROTOCOL_RX_BUFFER_SIZE] |
                               ((uint16_t)g_host_rx_ring[
                                   (g_host_ring_tail + 7U) %
                                   PROTOCOL_RX_BUFFER_SIZE] << 8);
                if (len > PROTOCOL_MAX_PAYLOAD) {
                    /* 恶意/错误 LEN：至少消费当前帧头第一字节后重同步，
                       保证解析进度；不刷新看门狗。 */
                    g_control_faults |= CONTROL_FAULT_RX_OVERFLOW;
                    g_host_ring_tail = (uint16_t)((g_host_ring_tail + 1U) %
                                                  PROTOCOL_RX_BUFFER_SIZE);
                    continue;
                }
                uint16_t total = (uint16_t)(AIMOTOR_BINARY_HEADER_SIZE +
                                            len + 2U);
                if (avail < total) {
                    return;   /* 半帧，等待后续数据 */
                }
                /* 整帧拷贝到栈缓冲后校验分发（帧最大 10+MAX_PAYLOAD 字节） */
                uint8_t frame_buf[AIMOTOR_BINARY_TOTAL(PROTOCOL_MAX_PAYLOAD)];
                for (uint16_t i = 0; i < total; i++) {
                    frame_buf[i] = g_host_rx_ring[
                        (uint16_t)((g_host_ring_tail + i) %
                                   PROTOCOL_RX_BUFFER_SIZE)];
                }
                /* 消费该帧（无论校验成败都不重放） */
                g_host_ring_tail = (uint16_t)(
                    (g_host_ring_tail + total) % PROTOCOL_RX_BUFFER_SIZE);
                uint16_t seq = (uint16_t)frame_buf[4] |
                               ((uint16_t)frame_buf[5] << 8);
                if (frame_buf[2] != AIMOTOR_BINARY_VERSION) {
                    BinarySendAck(seq, frame_buf[3], AIMOTOR_ACK_FORMAT);
                } else {
                    uint16_t got_crc = (uint16_t)frame_buf[8U + len] |
                                       ((uint16_t)frame_buf[9U + len] << 8);
                    /* CRC 覆盖 VER..payload 末尾 = 6+LEN 字节 */
                    if (Aimotor_CRC16(&frame_buf[2],
                                      (uint16_t)(6U + len)) != got_crc) {
                        BinarySendAck(seq, frame_buf[3], AIMOTOR_ACK_CRC);
                    } else {
                        /* 仅被真正接受的命令（REFRESH 结果）刷新看门狗 */
                        if (Aimotor_HandleBinaryFrame(frame_buf, len, total) ==
                            FRAME_ACCEPTED_REFRESH_WATCHDOG) {
                            Aimotor_RefreshHostWatchdog();
                        }
                    }
                }
                continue;
            }
            /* AA 后跟非 55：不是二进制帧，AA 按文本字节处理 */
        }

        /* 文本字节：聚合成行 */
        if (g_host_text_len == 0U) {
            g_host_text_activity_tick = now;
        }
        if (b0 != '\n' && b0 != '\r') {
            if (g_host_text_len < sizeof(g_host_text_buf) - 1U) {
                g_host_text_buf[g_host_text_len++] = (char)b0;
            }
        } else if (g_host_text_len > 0U) {
            g_host_text_buf[g_host_text_len] = '\0';
            uint16_t cmd_len = g_host_text_len;
            memcpy(g_host_cmd_buf, g_host_text_buf, cmd_len + 1U);
            g_host_cmd_len = cmd_len;
            g_host_cmd_ready = 1;
            memcpy((uint8_t *)g_mw_cmd_buf, g_host_text_buf, cmd_len + 1U);
            g_mw_cmd_len = cmd_len;
            g_mw_cmd_ready = 1;
            g_host_text_len = 0;
        }
        g_host_ring_tail = (uint16_t)((g_host_ring_tail + 1U) %
                                      PROTOCOL_RX_BUFFER_SIZE);
    }

    /* 文本行冲刷：无 \r\n 结尾的行在 50ms 无新数据后提交 */
    if (g_host_text_len > 0U &&
        (now - g_host_text_activity_tick) >= HOST_TEXT_LINE_TIMEOUT_MS) {
        g_host_text_buf[g_host_text_len] = '\0';
        uint16_t cmd_len = g_host_text_len;
        memcpy(g_host_cmd_buf, g_host_text_buf, cmd_len + 1U);
        g_host_cmd_len = cmd_len;
        g_host_cmd_ready = 1;
        memcpy((uint8_t *)g_mw_cmd_buf, g_host_text_buf, cmd_len + 1U);
        g_mw_cmd_len = cmd_len;
        g_mw_cmd_ready = 1;
        g_host_text_len = 0;
    }
}

/* 有效二进制命令被接受后刷新看门狗时间戳（仅 FRAME_ACCEPTED_REFRESH_WATCHDOG）。 */
void Aimotor_RefreshHostWatchdog(void)
{
    g_last_valid_frame_ms = HAL_GetTick();
}

/* 通信看门狗：ENABLED 状态下 250ms 无有效命令则立即置 STOPPED 状态
   （TARGET 即刻被拒），清除全部待执行目标，并启动全局停止序列停 12 轴。
   必须重新显式 ENABLE 才能恢复运动。使用无符号减法避免 tick 回绕。 */
void Aimotor_CommWatchdog(void)
{
    /* 控制序列进行中（ENABLE/STOP/DISABLE/回滚）不启动新序列：
       即使状态仍为 ENABLED（如冗余 ENABLE），也不允许看门狗抢占正在执行的
       安全序列（STOP/DISABLE 已在接受时置为非运动状态，双保险）。 */
    if (Aimotor_CtrlSeqActive()) {
        return;
    }
    if (g_control_state != CONTROL_ENABLED) {
        return;
    }
    uint32_t elapsed = HAL_GetTick() - g_last_valid_frame_ms;
    if (elapsed >= HOST_COMM_WATCHDOG_MS) {
        ClearAllPending();
        g_control_faults |= CONTROL_FAULT_COMM_TIMEOUT;
        g_control_state = CONTROL_STOPPED;
        CtrlSeqStart(CTRL_SEQ_STOP, 0, 0, 0);
    }
}

ControlState_t Aimotor_GetControlState(void)
{
    return g_control_state;
}

/* ======================================================================== */
/*                     strtol 位置解析                                        */
/* ======================================================================== */

static uint8_t ParsePosition(const char *text, int32_t *position)
{
    if (text == NULL || position == NULL) {
        return 0;
    }

    errno = 0;
    char *end = NULL;
    long value = strtol(text, &end, 10);

    if (text == end) {
        return 0;  /* 没有解析到数字 */
    }

    if (errno == ERANGE) {
        return 0;
    }

    if (value < AIMOTOR_POSITION_MIN ||
        value > AIMOTOR_POSITION_MAX) {
        return 0;
    }

    /* 数字后只允许空格、\r、\n、字符串结束，或兼容旧格式的 S 参数 */
    while (*end == ' ' || *end == '\t') {
        end++;
    }

    if (*end != '\0' && *end != '\r' && *end != '\n' &&
        *end != 'S' && *end != 's') {
        return 0;
    }

    *position = (int32_t)value;
    return 1;
}

/* ======================================================================== */
/*                       上位机命令解析                                     */
/* ======================================================================== */

/*
  * 命令格式:
  *   B<总线号> M<电机号> P<位置>            — 位置控制
  *   B<总线号> M<电机号> P<位置> S<速度>    — 位置控制(旧格式兼容，S被忽略)
  *   B<总线号> M<电机号> Q                  — 查询位置
  *   B<总线号> M<电机号> EN                 — 伺服使能
  *   B<总线号> M<电机号> HOME               — 单独回零
  *   B<总线号> M<电机号> GO                 — 手动模式
  *   B<总线号> TEST                         — 测试总线通信
  *   B<总线号> STOP                         — 停止总线所有电机
  *   ALL STOP                                — 紧急停止全部
  *
  *  返回值: 1=命令有效, 0=无效命令
  */
uint8_t HostCmd_Parse(const uint8_t *buf, uint16_t len, Aimotor_Cmd_t *cmd)
{
    if (!buf || !cmd || len < 4) return 0;

    memset(cmd, 0, sizeof(Aimotor_Cmd_t));
    cmd->type = AIMOTOR_CMD_NONE;

    const char *str = (const char *)buf;

    /* ALL STOP / ALL ENABLE / ALL DISABLE */
    if (strncmp(str, "ALL STOP", 8) == 0) {
        cmd->type    = AIMOTOR_CMD_STOP_ALL;
        cmd->bus_idx = 0xFF;   /* 特殊值表示全部 */
        cmd->valid   = 1;
        return 1;
    }
    if (strncmp(str, "ALL ENABLE", 10) == 0) {
#if AIMOTOR_TEXT_MOTION_ENABLED
        cmd->type    = AIMOTOR_CMD_ENABLE_ALL;
        cmd->bus_idx = 0xFF;
        cmd->valid   = 1;
        return 1;
#else
        /* 发布配置：文本使能命令关闭，防止绕过二进制协议控制状态机 */
        return 0;
#endif
    }
    if (strncmp(str, "ALL DISABLE", 11) == 0) {
        cmd->type    = AIMOTOR_CMD_DISABLE_ALL;
        cmd->bus_idx = 0xFF;
        cmd->valid   = 1;
        return 1;
    }

    /* 格式: B<num> ... */
    if (buf[0] != 'B' && buf[0] != 'b') return 0;
    cmd->bus_idx = (uint8_t)(buf[1] - '1');  /* 1-indexed → 0-indexed */
    if (cmd->bus_idx >= AIMOTOR_BUS_COUNT) return 0;

    /* B<num> STOP */
    if (strstr(str, "STOP") || strstr(str, "stop")) {
        cmd->type  = AIMOTOR_CMD_STOP_BUS;
        cmd->valid = 1;
        return 1;
    }

    /* B<num> TEST: 测试总线通信 */
    if (strstr(str, "TEST") || strstr(str, "test")) {
        cmd->type  = AIMOTOR_CMD_TEST;
        cmd->valid = 1;
        return 1;
    }

    /* 查找 M，后续命令都需要电机号 */
    const char *m_ptr = strchr(str, 'M');
    if (!m_ptr) m_ptr = strchr(str, 'm');
    if (!m_ptr) {
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR INVALID COMMAND\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        return 0;
    }

    cmd->motor_idx = (uint8_t)(m_ptr[1] - '1');
    if (cmd->motor_idx >= AIMOTOR_MOTORS_PER) {
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR INVALID MOTOR\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        return 0;
    }

    /* B<num> M<num> EN: 手动使能电机 */
    if (strstr(str, "EN") || strstr(str, "en")) {
        cmd->type  = AIMOTOR_CMD_ENABLE;
        cmd->valid = 1;
        return 1;
    }

    /* B<num> M<num> Q: 查询位置 */
    if (strstr(str, "Q") || strstr(str, "q")) {
        cmd->type  = AIMOTOR_CMD_QUERY;
        cmd->valid = 1;
        return 1;
    }

    /* B<num> M<num> GO: 手动模式 */
    if (strstr(str, "GO") || strstr(str, "go")) {
        cmd->type  = AIMOTOR_CMD_GO;
        cmd->valid = 1;
        return 1;
    }

    /* B<num> M<num> HOME: 单独回零 */
    if (strstr(str, "HOME") || strstr(str, "home")) {
        cmd->type  = AIMOTOR_CMD_HOME;
        cmd->valid = 1;
        return 1;
    }

    /* B<num> M<num> P<num> [S<num>]: 位置控制 */
    const char *p_ptr = strchr(m_ptr, 'P');
    if (!p_ptr) p_ptr = strchr(m_ptr, 'p');
    if (!p_ptr) {
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR INVALID COMMAND\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        return 0;
    }

    int32_t pos;
    if (!ParsePosition(p_ptr + 1, &pos)) {
        /* ParsePosition 失败：可能是格式错误或超范围 */
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR INVALID POSITION\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        return 0;
    }

    /* 范围二次确认（ParsePosition 内已检查，此处保底） */
    if (pos < AIMOTOR_POSITION_MIN || pos > AIMOTOR_POSITION_MAX) {
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR POSITION OUT OF RANGE\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        return 0;
    }

    cmd->type     = AIMOTOR_CMD_MOVE;
    cmd->position = pos;
    cmd->valid    = 1;
    return 1;
}

/* ======================================================================== */
/*                       上位机命令执行                                     */
/* ======================================================================== */

void HostCmd_Execute(const Aimotor_Cmd_t *cmd)
{
    if (!cmd || !cmd->valid) return;

    uint8_t b, m;
    int n;

    /* 文本命令不刷新 ROS 2 通信看门狗（仅二进制有效命令刷新）。 */

    switch (cmd->type) {

    case AIMOTOR_CMD_STOP_ALL:
    case AIMOTOR_CMD_STOP_BUS:
        /* 文本 STOP：统一走异步全局安全停止序列（非阻塞、每台回包确认），
           不再逐台阻塞发送且不确认。B<num> STOP 与 ALL STOP 均停止双臂 12 轴，
           保证与二进制 STOP 语义一致。 */
        /* STOP 最高优先级：CtrlSeqStart 会抢占 ENABLE/DISABLE/ROLLBACK；
           不在文本路径以 CONTROL BUSY 拒绝安全停止。 */
        CtrlSeqStart(CTRL_SEQ_STOP, 0, 0, 0);
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK STOP: ACCEPTED\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        break;

    case AIMOTOR_CMD_TEST:
        /* 旧 B TEST 会绕过统一控制序列直接 Servo On，永久关闭该危险入口。 */
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR B TEST DISABLED: use binary ENABLE\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        break;

    case AIMOTOR_CMD_ENABLE:
        /* 单轴文本使能不再支持：无法在文本路径完成回包确认，直接写 enabled=1
           会与 STATE 的“实际确认使能”语义冲突。统一使用二进制 ENABLE 或
           ALL ENABLE（异步序列 + 回包确认）完成使能。 */
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR: single-axis EN disabled, use ALL ENABLE\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        break;

    case AIMOTOR_CMD_QUERY:
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "POS B%d M%d %ld\r\n",
                     b + 1, m + 1,
                     (long)aimotor_motors[b][m].actual_position);
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        break;

    case AIMOTOR_CMD_GO:
#if AIMOTOR_TEXT_MOTION_ENABLED
        if (g_control_state != CONTROL_ENABLED) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL DENIED\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        if (aimotor_motors[b][m].fault) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        /* 统一交给生产状态机 STOP→WRITE→TRIGGER→QUERY，等待并校验每个回包；
           不再使用累计 100ms 的 HAL_Delay 阻塞主循环。 */
        aimotor_motors[b][m].cmd_pending = 1;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK B%d M%d GO: ACCEPTED\r\n", b + 1, m + 1);
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
#else
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR TEXT MOTION DISABLED\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
#endif
        break;

    case AIMOTOR_CMD_HOME:
#if AIMOTOR_TEXT_MOTION_ENABLED
        if (g_control_state != CONTROL_ENABLED) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL DENIED\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        if (aimotor_motors[b][m].fault) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        aimotor_motors[b][m].target_position = 0;
        aimotor_motors[b][m].cmd_pending = 1;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK B%d M%d HOME\r\n", b + 1, m + 1);
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
#else
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR TEXT MOTION DISABLED\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
#endif
        break;

    case AIMOTOR_CMD_MOVE:
#if AIMOTOR_TEXT_MOTION_ENABLED
        if (g_control_state != CONTROL_ENABLED) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL DENIED\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        if (aimotor_motors[b][m].fault) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        aimotor_motors[b][m].target_position = cmd->position;
        aimotor_motors[b][m].cmd_pending = 1;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK B%d M%d P%ld: ACCEPTED\r\n",
                     b + 1, m + 1, (long)cmd->position);
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
#else
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR TEXT MOTION DISABLED\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
#endif
        break;

    case AIMOTOR_CMD_ENABLE_ALL:
        if (Aimotor_CtrlSeqActive()) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL BUSY\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        if (g_control_state == CONTROL_ENABLED) {
            if (!Aimotor_AllAxesConfirmedEnabled()) {
                n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                             "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
            } else {
                n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                             "OK ALL ENABLE: ALREADY ENABLED\r\n");
            }
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        if ((g_control_faults & CONTROL_FAULT_INTERNAL) != 0U) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL FAULT\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        CtrlSeqStart(CTRL_SEQ_ENABLE, 0, 0, 0);
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK ALL ENABLE: ACCEPTED\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        break;

    case AIMOTOR_CMD_DISABLE_ALL:
        if (Aimotor_CtrlSeqActive()) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL BUSY\r\n");
            Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
            break;
        }
        CtrlSeqStart(CTRL_SEQ_DISABLE, 0, 0, 0);
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK ALL DISABLE: ACCEPTED\r\n");
        Host_ReplyBytes((uint8_t *)g_host_tx_buf, (uint16_t)n);
        break;

    case AIMOTOR_CMD_NONE:
    default:
        break;
    }
}

/* ======================================================================== */
/*                       协议层初始化                                        */
/* ======================================================================== */

/* 协议层初始化：启动 USART1 DMA 空闲接收（原 main.c 初始化代码迁入） */
void Host_ProtocolInit(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_host_dma_buf, sizeof(g_host_dma_buf));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/* 上位机回复等待上限：STATE 帧 70 字节 @115200 约 6ms，20ms 覆盖两帧排队 */
#define HOST_TX_WAIT_MS 20

/* 非阻塞回复发送（USART1 中断方式）：上一帧未发完时有界等待，异常时退化为
   阻塞发送。所有上位机回复（ACK/STATE/文本）统一走此通道，5ms 调度循环
   不再被 STATE 帧的 ~6ms 阻塞发送拖住。仅在主循环上下文调用。
   数据先拷入持久静态缓冲 g_host_tx_async_buf，避免调用方局部栈数组在异步
   发送期间失效（HAL_UART_Transmit_IT 不复制数据，只保存源地址）。 */
void Host_ReplyBytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > HOST_TX_BUF_SIZE) {
        return;
    }

    /* 必须等上一帧发完（UART 回到 READY）才能覆盖 g_host_tx_async_buf；
       超时仍不 READY 则丢弃本帧，绝不覆盖正在发送的缓冲。 */
    uint32_t start = HAL_GetTick();
    while (huart1.gState != HAL_UART_STATE_READY) {
        if ((HAL_GetTick() - start) >= HOST_TX_WAIT_MS) {
            return;
        }
    }

    memcpy(g_host_tx_async_buf, data, len);

    if (HAL_UART_Transmit_IT(&huart1, g_host_tx_async_buf, len) != HAL_OK) {
        /* IT 启动失败时允许退化为 blocking TX（缓冲已是持久副本）。 */
        (void)HAL_UART_Transmit(&huart1,
                                g_host_tx_async_buf,
                                len,
                                HOST_TX_WAIT_MS);
    }
}


/* USER CODE END 0 */
