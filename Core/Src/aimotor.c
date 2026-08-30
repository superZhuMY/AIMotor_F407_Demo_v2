/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : aimotor.c
  * @brief          : AI Motor multi-bus RS485 Modbus RTU control module
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "aimotor.h"
#include "mwmotor.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>

/* USER CODE BEGIN 0 */

/* ======================================================================== */
/*               编译期配置确认（打印到编译日志，防止宏未生效）                  */
/* ======================================================================== */

#define AIMOTOR_XSTR_(x) #x
#define AIMOTOR_XSTR(x)  AIMOTOR_XSTR_(x)
#pragma message("AIMOTOR_DRY_RUN             = " AIMOTOR_XSTR(AIMOTOR_DRY_RUN))
#pragma message("AIMOTOR_TEXT_MOTION_ENABLED = " AIMOTOR_XSTR(AIMOTOR_TEXT_MOTION_ENABLED))
#pragma message("AIMOTOR_SELF_TEST           = " AIMOTOR_XSTR(AIMOTOR_SELF_TEST))

#if AIMOTOR_SELF_TEST && !AIMOTOR_DRY_RUN
#error "AIMOTOR_SELF_TEST requires AIMOTOR_DRY_RUN=1"
#endif

/* ======================================================================== */
/*                          全局变量定义                                      */
/* ======================================================================== */

Aimotor_Bus_t aimotor_buses[AIMOTOR_BUS_COUNT];
Aimotor_t     aimotor_motors[AIMOTOR_BUS_COUNT][AIMOTOR_MOTORS_PER];

/* 12 轴轴序静态断言：L_J1..L_J6, R_J1..R_J6（位图统一 side*6+joint） */
_Static_assert(AIMOTOR_BUS_COUNT * (AIMOTOR_MOTORS_PER + MW_MOTORS_PER) == 12,
               "12 axis bitmap requires 2 arms x 6 joints");

/* DRY_RUN 计数与 mock 记录（真实发送计数：DRY_RUN 下恒为 0） */
volatile uint32_t g_dry_run_ai_tx_count = 0;
volatile uint32_t g_dry_run_mw_tx_count = 0;
volatile DryRunLogEntry_t g_dry_run_log[AIMOTOR_DRY_RUN_LOG_MAX];
volatile uint8_t  g_dry_run_log_index = 0;

/* mock：记录“将发送”的命令帧（DRY_RUN 下真实发送函数被短路） */
void Aimotor_DryRunLog(uint8_t is_mw, const uint8_t *data, uint16_t len)
{
    volatile DryRunLogEntry_t *e = &g_dry_run_log[g_dry_run_log_index];
    uint16_t n = (len < AIMOTOR_DRY_RUN_FRAME_MAX) ? len : AIMOTOR_DRY_RUN_FRAME_MAX;
    for (uint16_t i = 0; i < n; i++) e->data[i] = data[i];
    e->len = (uint8_t)n;
    e->is_mw = is_mw;
    g_dry_run_log_index = (uint8_t)((g_dry_run_log_index + 1U) %
                                    AIMOTOR_DRY_RUN_LOG_MAX);
}

/* 引用上位机主机命令缓冲（在 main.c 中定义） */
extern char g_host_cmd_buf[64];
extern char g_host_tx_buf[96];
extern volatile uint16_t g_host_cmd_len;
extern volatile uint8_t  g_host_cmd_ready;
extern uint8_t g_host_rx_buf[64];
extern volatile uint16_t g_host_rx_len;
extern volatile uint8_t  g_host_rx_ready;

/* ======================================================================== */
/*                上位机流接收环形缓冲（拆包/粘包/CRC 重同步）                  */
/* ======================================================================== */

static uint8_t  g_host_rx_ring[PROTOCOL_RX_BUFFER_SIZE];
static volatile uint16_t g_host_ring_head = 0;  /* 下一个写入位置 */
static volatile uint16_t g_host_ring_tail = 0;  /* 下一个读取位置 */

/* 文本行提取缓冲（流解析 → 现有命令缓冲） */
static char     g_host_text_buf[64];
static uint16_t g_host_text_len = 0;
static uint32_t g_host_text_activity_tick = 0;

/* ======================================================================== */
/*                      安全层：控制状态机 + 通信看门狗                        */
/* ======================================================================== */

static ControlState_t g_control_state = CONTROL_DISABLED;
static uint8_t        g_control_faults = CONTROL_FAULT_NONE;
static uint32_t       g_last_valid_frame_ms = 0;  /* 上次有效命令时间 */

/* 异步控制序列（ENABLE/STOP/DISABLE/上电安全停止） */
typedef enum {
    STEP_AI_SERVO_ON,   /* 总线使能 0x0303=1 */
    STEP_AI_SERVO_OFF,  /* 总线断开 0x0303=0 */
    STEP_AI_STOP,       /* 停止 0x0305=0 */
    STEP_MW_RUN,        /* 0x88 运行 */
    STEP_MW_CLOSE       /* 0x80 关闭 */
} SeqOp_t;

/* 每个序列最多步骤数（DISABLE = 6 停止 + 6 关闭 + 6 ServoOff = 18） */
#define CTRL_SEQ_MAX_STEPS 18
typedef struct {
    SeqOp_t op;
    uint8_t side;
    uint8_t axis;       /* 该 side 的轴序号（AI 0..2 / MW 0..2） */
} SeqStep_t;

typedef struct {
    CtrlSeqKind_t kind;
    uint8_t  nsteps;        /* 步骤总数 */
    uint8_t  step;          /* 当前步骤索引 */
    uint8_t  phase;         /* 0=发送命令，1=等待回包 */
    uint32_t tx_tick;       /* 当前步骤发送时间 */
    uint8_t  retry;         /* 当前步骤重试计数（仅本步骤） */
    uint16_t failed_mask;   /* 失败轴位图（side*6+joint，12bit） */
    uint8_t  rollback;      /* 1=正在执行回滚序列 */
    uint16_t pending_seq;   /* 待回发 ACK 的请求 SEQ */
    uint8_t  pending_cmd;   /* 待回发 ACK 的 CMD */
    uint8_t  active;        /* 1=序列完成后需回发 ACK（二进制命令） */
    SeqStep_t steps[CTRL_SEQ_MAX_STEPS];   /* 步骤表 */
} CtrlSeq_t;

static CtrlSeq_t g_ctrl_seq;

/* 自检 TX 捕获（AIMOTOR_SELF_TEST：把 ACK/STATE 输出重定向到捕获缓冲） */
#if AIMOTOR_SELF_TEST
static uint8_t  g_test_capture_tx = 0;
static uint8_t  g_test_tx_log[1024];
static uint16_t g_test_tx_len = 0;
static uint8_t  g_test_reply_wait = 0;   /* 1: DRY_RUN 下序列也等待回包（测重试/超时） */
static int      g_test_pass = 0;
static int      g_test_fail = 0;
static uint8_t  g_test_seq_send_count = 0;
static uint16_t g_test_seq_ai_stop_mask = 0;
static uint16_t g_test_seq_ai_off_mask = 0;
static uint16_t g_test_seq_mw_close_mask = 0;
#endif

/* 静态函数声明 */
static uint8_t  ProcessRx(uint8_t bus_idx, uint8_t motor_idx);
static void     HostCmd_Execute(const Aimotor_Cmd_t *cmd);
static uint8_t  ParsePosition(const char *text, int32_t *position);
static void     Aimotor_EncodeInt32CDAB(int32_t value, uint8_t out[4]);
static int32_t  Aimotor_DecodeInt32CDAB(const uint8_t data[4]);
static uint8_t  Aimotor_RequestTimedOut(const Aimotor_t *motor);
static int32_t BinaryReadI32(const uint8_t *p);
static void BinarySendAck(uint16_t seq, uint8_t cmd, uint8_t result);
static void Aimotor_SendState(uint16_t seq);
static uint8_t BinaryTargetValidateAndDispatch(uint8_t side,
                                               const int32_t joints[6]);
static FrameDispatchResult_t Aimotor_HandleBinaryFrame(const uint8_t *f,
                                                       uint16_t len,
                                                       uint16_t total_len);
static void CtrlSeqTick(void);
static void CtrlSeqStart(CtrlSeqKind_t kind, uint16_t seq, uint8_t cmd,
                         uint8_t has_ack);
static void CtrlSeqSetAxisEnabled(uint8_t confirmed);
static void CtrlSeqClearAxisFault(void);
static uint8_t CtrlSeqBuildDisableSteps(void);
static void CtrlSeqBuildRollback(void);
static void ClearAllPending(void);
static uint8_t Aimotor_ArmHasLatchedFault(uint8_t side);
static uint8_t Aimotor_AllAxesConfirmedEnabled(void);

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

static int32_t Aimotor_DecodeInt32CDAB(const uint8_t data[4])
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

static int32_t BinaryReadI32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

/* ACK 帧：AA 55 | VER | 0x80 | SEQ(2) | LEN=4 | seq(2)+cmd(1)+result(1) | CRC(2)
   LEN 与实际 payload(4) 严格一致，总长 10+4=14，CRC 位于 [12..13]。 */
static void BinarySendAck(uint16_t seq, uint8_t cmd, uint8_t result)
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
    HAL_UART_Transmit(&huart1, out, sizeof(out), 100);
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
    HAL_UART_Transmit(&huart1, out, n, 100);
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

    /* Phase 2：全部合法，统一下发 */
    for (uint8_t i = 0; i < 3; i++) {
        aimotor_motors[side][i].target_position = (int32_t)ai_pulse[i];
        aimotor_motors[side][i].cmd_pending = 1;
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
            memcpy((uint8_t *)g_host_rx_buf, g_host_text_buf, cmd_len + 1U);
            g_host_rx_len = cmd_len;
            g_host_rx_ready = 1;
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
        memcpy((uint8_t *)g_host_rx_buf, g_host_text_buf, cmd_len + 1U);
        g_host_rx_len = cmd_len;
        g_host_rx_ready = 1;
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

/* 清空全部待执行目标与重试任务（STOP/DISABLE/看门狗/新事务开始时调用） */
static void ClearAllPending(void)
{
    for (uint8_t b = 0; b < AIMOTOR_BUS_COUNT; b++) {
        for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
            aimotor_motors[b][m].cmd_pending = 0;
            aimotor_motors[b][m].retry_count = 0;
            aimotor_motors[b][m].step = MOTOR_STEP_IDLE;
            aimotor_motors[b][m].actual_valid = 0;
            aimotor_motors[b][m].rx_ready = 0;
        }
    }
    for (uint8_t side = 0; side < MW_BUS_COUNT; side++) {
        mw_coupled[side].pending = 0;
        mw_coupled[side].arrived = 0;
        mw_coupled[side].fault = 0;
        mw_coupled[side].step = MW_COUPLED_IDLE;
        mw_coupled[side].retry_count = 0;
        for (uint8_t m = 0; m < MW_MOTORS_PER; m++) {
            mw_motors[side][m].cmd_pending = 0;
            mw_motors[side][m].retry_count = 0;
            mw_motors[side][m].step = MW_STEP_IDLE;
            mw_motors[side][m].actual_valid = 0;
        }
        mw_buses[side].rx_ready = 0;
        mw_buses[side].rx_len = 0;
        MW_IdlePollReset(side);
    }
}

/* ======================================================================== */
/*              异步控制序列（ENABLE/STOP/DISABLE/上电安全停止）              */
/* ======================================================================== */

static void CtrlSeqFinish(void)
{
    uint8_t fail = (g_ctrl_seq.failed_mask != 0U);
    uint8_t result = AIMOTOR_ACK_OK;

    switch (g_ctrl_seq.kind) {
    case CTRL_SEQ_STARTUP_SAFE_STOP:
        /* 上电安全停止：失败轴置未确认停止故障位；状态保持 DISABLED */
        if (fail) {
            g_control_faults |= CONTROL_FAULT_UNCONFIRMED_STOP;
        }
        g_control_state = CONTROL_DISABLED;
        break;

    case CTRL_SEQ_ENABLE:
        if (fail) {
            /* ENABLE 必要电机失败：不进入 ENABLED，置故障并进入 FAULT；
               对已确认成功的轴执行回滚（Servo Off / 0x80），避免部分使能。 */
            g_control_faults |= CONTROL_FAULT_ENABLE_FAILED;
            g_control_state = CONTROL_FAULT;
            CtrlSeqBuildRollback();
            return;   /* 回滚序列继续推进，ACK 延迟到回滚完成后 */
        }
        /* 故障清除矩阵：可通过重新使能恢复的位在本次成功使能时清除。
           CONTROL_FAULT_INTERNAL 表示内部状态损坏，只能通过复位清除。 */
        g_control_faults &= (uint8_t)~(CONTROL_FAULT_COMM_TIMEOUT |
                                       CONTROL_FAULT_RX_OVERFLOW |
                                       CONTROL_FAULT_ENABLE_FAILED |
                                       CONTROL_FAULT_UNCONFIRMED_STOP);
        if ((g_control_faults & CONTROL_FAULT_INTERNAL) != 0U) {
            g_control_state = CONTROL_FAULT;
            result = AIMOTOR_ACK_CTRL_FAILED;
        } else {
            g_control_state = CONTROL_ENABLED;
        }
        break;

    case CTRL_SEQ_ROLLBACK:
        /* 回滚：成功轴已全部关闭。ENABLE 失败 ACK 在回滚完成后回发。
           注意：不把回滚本身当作一次独立刷新。 */
        if (fail) {
            g_control_faults |= CONTROL_FAULT_UNCONFIRMED_STOP;
        }
        result = AIMOTOR_ACK_CTRL_FAILED;
        break;

    case CTRL_SEQ_STOP:
        if (fail) {
            g_control_faults |= CONTROL_FAULT_UNCONFIRMED_STOP;
            result = AIMOTOR_ACK_CTRL_FAILED;   /* 部分失败 */
        }
        g_control_state = CONTROL_STOPPED;
        break;

    case CTRL_SEQ_DISABLE:
        if (fail) {
            g_control_faults |= CONTROL_FAULT_UNCONFIRMED_STOP;
            result = AIMOTOR_ACK_CTRL_FAILED;
        }
        g_control_state = CONTROL_DISABLED;
        break;

    default:
        g_control_faults |= CONTROL_FAULT_INTERNAL;
        break;
    }

    /* 回发延迟 ACK（仅二进制命令；ACK 语义=序列完成后的真实结果） */
    if (g_ctrl_seq.active) {
        BinarySendAck(g_ctrl_seq.pending_seq, g_ctrl_seq.pending_cmd, result);
    }

    /* 通信看门狗刷新时机：ENABLE/STOP/DISABLE 只在序列**成功完成**时刷新
       （保证进入 ENABLED 前时间戳新鲜，避免旧时间戳立刻触发看门狗）。
       失败（0x0A）不刷新；文本命令（active=0）不刷新；看门狗自触发的
       STOP（active=0）不刷新。 */
    if (g_ctrl_seq.active && result == AIMOTOR_ACK_OK) {
        Aimotor_RefreshHostWatchdog();
    }

    g_ctrl_seq.kind = CTRL_SEQ_NONE;
    g_ctrl_seq.active = 0;
}

/* 构建全轴安全停止+禁用步骤表（DISABLE 与 ENABLE 失败回滚共用）：
   AI 6×StopMotion(0x0305=0) + MW 6×0x80 + AI 6×ServoOff(0x0303=0) = 18 步 */
static uint8_t CtrlSeqBuildDisableSteps(void)
{
    uint8_t n = 0;
    for (uint8_t b = 0; b < AIMOTOR_BUS_COUNT; b++) {
        for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
            g_ctrl_seq.steps[n].op = STEP_AI_STOP; g_ctrl_seq.steps[n].side = b;
            g_ctrl_seq.steps[n].axis = m; n++;
        }
    }
    for (uint8_t b = 0; b < MW_BUS_COUNT; b++) {
        for (uint8_t m = 0; m < MW_MOTORS_PER; m++) {
            g_ctrl_seq.steps[n].op = STEP_MW_CLOSE; g_ctrl_seq.steps[n].side = b;
            g_ctrl_seq.steps[n].axis = m; n++;
        }
    }
    for (uint8_t b = 0; b < AIMOTOR_BUS_COUNT; b++) {
        for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
            g_ctrl_seq.steps[n].op = STEP_AI_SERVO_OFF; g_ctrl_seq.steps[n].side = b;
            g_ctrl_seq.steps[n].axis = m; n++;
        }
    }
    return n;
}

/* 构建步骤表并启动序列。STOP 可抢占其它序列；ENABLE/DISABLE 在序列忙时拒绝。
   抢占竞态处理（P0）：
   - STOP 执行中收到第二条 STOP：不重建（避免总线命令重叠）。若已有等待 ACK
     的 STOP（active=1），第二条立即回 CTRL_BUSY(0x09)，保持原 STOP 的 ACK
     不丢失；若当前 STOP 由看门狗/文本发起（active=0），二进制 STOP 挂接 ACK
     到当前序列。
   - STOP 抢占 ENABLE/DISABLE/回滚：被抢占命令若等待 ACK，先回 0x0A 告知其
     未完成，再重建全轴安全停止（回滚只覆盖部分轴，STOP 必须真正覆盖 12 轴）。 */
static void CtrlSeqStart(CtrlSeqKind_t kind, uint16_t seq, uint8_t cmd,
                         uint8_t has_ack)
{
    if (kind == CTRL_SEQ_STOP || kind == CTRL_SEQ_STARTUP_SAFE_STOP) {
        if (g_ctrl_seq.kind == CTRL_SEQ_STOP ||
            g_ctrl_seq.kind == CTRL_SEQ_STARTUP_SAFE_STOP) {
            /* 已在停止类序列中：不重建 */
            if (has_ack) {
                if (g_ctrl_seq.active) {
                    /* 已有一条二进制 STOP 等待 ACK：新 STOP 立即回 BUSY，
                       不覆盖原 pending_seq/pending_cmd */
                    BinarySendAck(seq, cmd, AIMOTOR_ACK_CTRL_BUSY);
                } else {
                    /* 看门狗/文本发起的 STOP（无 ACK 等待）：挂接二进制 STOP */
                    g_ctrl_seq.active = 1;
                    g_ctrl_seq.pending_seq = seq;
                    g_ctrl_seq.pending_cmd = cmd;
                }
            }
            return;
        }
        /* 抢占其它序列（ENABLE/DISABLE/ROLLBACK/STARTUP 之外）：
           被抢占命令若等待 ACK，先回 0x0A 告知其未完成 */
        if (g_ctrl_seq.kind != CTRL_SEQ_NONE) {
            if (g_ctrl_seq.active) {
                BinarySendAck(g_ctrl_seq.pending_seq, g_ctrl_seq.pending_cmd,
                              AIMOTOR_ACK_CTRL_FAILED);
            }
            g_ctrl_seq.kind = CTRL_SEQ_NONE;
            g_ctrl_seq.active = 0;
            ClearAllPending();
        }
        /* 注意：这里**不**清除 CONTROL_FAULT_COMM_TIMEOUT——
           看门狗超时本身通过本路径执行全局 STOP，超时故障位必须在 STATE
           中保持，直到显式 ENABLE 成功才清除（见 CtrlSeqFinish）。 */
    }

    /* DISABLE：接受时立即清除全部旧运动事务（AI/MW 耦合/J4/空闲读取），
       防止序列完成后旧事务在 DISABLED 状态继续发送命令。
       （ROLLBACK 在 CtrlSeqBuildRollback 内同样清除。） */
    if (kind == CTRL_SEQ_DISABLE) {
        ClearAllPending();
    }

    /* 接受 STOP/DISABLE 时立即进入非运动过渡状态：
       - 看门狗只会在 CONTROL_ENABLED 下触发；提前置 STOPPED/DISABLED 可防止
         同轮看门狗超时抢占本序列（否则 DISABLE 完成前状态仍为 ENABLED，
         旧时间戳可能触发 STOP 抢占，AI Servo Off 不再执行）。
       - ENABLE 保持原状态直到成功（不提前置 ENABLED）。 */
    if (kind == CTRL_SEQ_STOP || kind == CTRL_SEQ_STARTUP_SAFE_STOP) {
        g_control_state = CONTROL_STOPPED;
    } else if (kind == CTRL_SEQ_DISABLE) {
        g_control_state = CONTROL_DISABLED;
    }

    g_ctrl_seq.kind = kind;
    g_ctrl_seq.step = 0;
    g_ctrl_seq.phase = 0;
    g_ctrl_seq.retry = 0;
    g_ctrl_seq.tx_tick = 0;
    g_ctrl_seq.failed_mask = 0;
    g_ctrl_seq.rollback = 0;
    g_ctrl_seq.pending_seq = seq;
    g_ctrl_seq.pending_cmd = cmd;
    g_ctrl_seq.active = has_ack;
    g_ctrl_seq.nsteps = 0;

    /* 构建步骤表 */
    uint8_t n = 0;
    switch (kind) {
    case CTRL_SEQ_ENABLE:
        for (uint8_t b = 0; b < AIMOTOR_BUS_COUNT; b++) {
            for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
                g_ctrl_seq.steps[n].op = STEP_AI_SERVO_ON; g_ctrl_seq.steps[n].side = b;
                g_ctrl_seq.steps[n].axis = m; n++;
            }
        }
        for (uint8_t b = 0; b < MW_BUS_COUNT; b++) {
            for (uint8_t m = 0; m < MW_MOTORS_PER; m++) {
                g_ctrl_seq.steps[n].op = STEP_MW_RUN; g_ctrl_seq.steps[n].side = b;
                g_ctrl_seq.steps[n].axis = m; n++;
            }
        }
        break;

    case CTRL_SEQ_STOP:
        for (uint8_t b = 0; b < AIMOTOR_BUS_COUNT; b++) {
            for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
                g_ctrl_seq.steps[n].op = STEP_AI_STOP; g_ctrl_seq.steps[n].side = b;
                g_ctrl_seq.steps[n].axis = m; n++;
            }
        }
        for (uint8_t b = 0; b < MW_BUS_COUNT; b++) {
            for (uint8_t m = 0; m < MW_MOTORS_PER; m++) {
                g_ctrl_seq.steps[n].op = STEP_MW_CLOSE; g_ctrl_seq.steps[n].side = b;
                g_ctrl_seq.steps[n].axis = m; n++;
            }
        }
        break;

    case CTRL_SEQ_DISABLE:
        /* 全轴安全停止+禁用：AI StopMotion + MW 0x80 + AI Servo Off */
        n = CtrlSeqBuildDisableSteps();
        break;

    case CTRL_SEQ_STARTUP_SAFE_STOP:
        /* 上电安全停止：AI Servo Off 6 台 + MW 0x80 6 台 */
        for (uint8_t b = 0; b < AIMOTOR_BUS_COUNT; b++) {
            for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
                g_ctrl_seq.steps[n].op = STEP_AI_SERVO_OFF; g_ctrl_seq.steps[n].side = b;
                g_ctrl_seq.steps[n].axis = m; n++;
            }
        }
        for (uint8_t b = 0; b < MW_BUS_COUNT; b++) {
            for (uint8_t m = 0; m < MW_MOTORS_PER; m++) {
                g_ctrl_seq.steps[n].op = STEP_MW_CLOSE; g_ctrl_seq.steps[n].side = b;
                g_ctrl_seq.steps[n].axis = m; n++;
            }
        }
        break;

    default:
        g_ctrl_seq.kind = CTRL_SEQ_NONE;
        g_ctrl_seq.active = 0;
        g_control_faults |= CONTROL_FAULT_INTERNAL;
        return;
    }
    g_ctrl_seq.nsteps = n;
}

/* ENABLE 部分失败回滚：对**全部 12 轴**执行安全停止+禁用。
   依据：ENABLE 失败时无法区分"命令已执行但应答丢失"与"命令未执行"的轴——
   没有成功应答只能说明状态未知，不能证明驱动未执行。因此对全部 AI
   （StopMotion + ServoOff）与全部 MW（0x80）执行安全禁用，而不是只处理
   确认成功轴。同时清除全部旧运动事务，防止恢复后旧目标继续执行。 */
static void CtrlSeqBuildRollback(void)
{
    g_ctrl_seq.kind = CTRL_SEQ_ROLLBACK;
    g_ctrl_seq.step = 0;
    g_ctrl_seq.phase = 0;
    g_ctrl_seq.retry = 0;
    g_ctrl_seq.failed_mask = 0;   /* 回滚自身失败单独累计 */
    g_ctrl_seq.rollback = 1;
    ClearAllPending();            /* 清除旧运动事务 */
    g_ctrl_seq.nsteps = CtrlSeqBuildDisableSteps();

    /* 防御：无步骤（正常不会发生）则直接经 CtrlSeqFinish 回发 ENABLE 失败 ACK */
    if (g_ctrl_seq.nsteps == 0U) {
        g_ctrl_seq.rollback = 0;
        CtrlSeqFinish();
    }
}

/* 校验当前步骤的回包（AI：0x06/0x10 写应答；MW：0x88/0x80 应答）。
   AI 应答必须校验：站号、功能码、寄存器地址、CRC，以及**写入值/寄存器数量**
   （ServoOn 应答值 0x0001，ServoOff 应答值 0x0000，Stop 应答寄存器数量 1），
   防止旧回包或不对应当前操作的回包被误判为成功。 */
static uint8_t CtrlSeqStepReplyOk(void)
{
    const SeqStep_t *s = &g_ctrl_seq.steps[g_ctrl_seq.step];

    if (s->op == STEP_MW_RUN || s->op == STEP_MW_CLOSE) {
        MW_Bus_t *bus = &mw_buses[s->side];
        uint8_t cmd = (s->op == STEP_MW_RUN) ? 0x88U : 0x80U;
        if (!bus->rx_ready) return 0;
        uint8_t ok = MW_ReplyOk(s->side, mw_motors[s->side][s->axis].id, cmd);
        bus->rx_ready = 0;
        return ok;
    }

    /* AI 步骤：0x06 写单寄存器 / 0x10 写多寄存器应答 */
    Aimotor_t *motor = &aimotor_motors[s->side][s->axis];
    if (!motor->rx_ready) return 0;
    motor->rx_ready = 0;
    uint8_t len = motor->rx_len;
    const uint8_t *rx = motor->rx_buf;
    if (len != 8U) return 0;
    if (rx[0] != motor->slave_id) return 0;
    if (Aimotor_CRC16(rx, 6U) != ((uint16_t)rx[7] << 8 | rx[6])) return 0;
    if (s->op == STEP_AI_SERVO_ON || s->op == STEP_AI_SERVO_OFF) {
        /* 0x06 应答：slave, 06, reg_hi, reg_lo, val_hi, val_lo, crc(2) */
        if (rx[1] != 0x06U || rx[2] != 0x03U || rx[3] != 0x03U) return 0;
        if (rx[4] != 0x00U) return 0;
        if (s->op == STEP_AI_SERVO_ON) {
            if (rx[5] != 0x01U) return 0;   /* 必须确认写入 1 */
        } else {
            if (rx[5] != 0x00U) return 0;   /* 必须确认写入 0 */
        }
    } else { /* STEP_AI_STOP */
        /* 0x10 应答：slave, 10, reg_hi, reg_lo, qty_hi, qty_lo, crc(2) */
        if (rx[1] != 0x10U || rx[2] != 0x03U || rx[3] != 0x05U) return 0;
        if (rx[4] != 0x00U || rx[5] != 0x01U) return 0;   /* 寄存器数量=1 */
    }
    return 1;
}

/* 发送当前步骤命令（真实路径或 DRY_RUN mock 记录由各发送函数内部处理） */
static void CtrlSeqSendCmd(void)
{
    const SeqStep_t *s = &g_ctrl_seq.steps[g_ctrl_seq.step];
#if AIMOTOR_SELF_TEST
    /* 记录生产 CtrlSeqSendCmd 真正走过的回滚步骤，避免仅检查软件 enabled
       而把“没有发关闭命令”误判为回滚成功。重试会增加 send_count。 */
    if (g_test_seq_send_count < 0xFFU) g_test_seq_send_count++;
    if (s->op == STEP_AI_STOP) {
        g_test_seq_ai_stop_mask |= (uint16_t)(1U << (s->side * 3U + s->axis));
    } else if (s->op == STEP_AI_SERVO_OFF) {
        g_test_seq_ai_off_mask |= (uint16_t)(1U << (s->side * 3U + s->axis));
    } else if (s->op == STEP_MW_CLOSE) {
        g_test_seq_mw_close_mask |= (uint16_t)(1U << (s->side * 3U + s->axis));
    }
#endif
    switch (s->op) {
    case STEP_AI_SERVO_ON:  Aimotor_ServoOn(s->side, aimotor_motors[s->side][s->axis].slave_id); break;
    case STEP_AI_SERVO_OFF: Aimotor_ServoOff(s->side, aimotor_motors[s->side][s->axis].slave_id); break;
    case STEP_AI_STOP:      Aimotor_StopMotion(s->side, aimotor_motors[s->side][s->axis].slave_id); break;
    case STEP_MW_RUN:       MW_Send88(s->side, mw_motors[s->side][s->axis].id); break;
    case STEP_MW_CLOSE:     MW_Send80(s->side, mw_motors[s->side][s->axis].id); break;
    default: break;
    }
}

/* 按步骤类型更新轴的“实际确认使能状态”（仅回包确认后 / DRY_RUN 立即确认） */
static void CtrlSeqSetAxisEnabled(uint8_t confirmed)
{
    const SeqStep_t *s = &g_ctrl_seq.steps[g_ctrl_seq.step];
    switch (s->op) {
    case STEP_AI_SERVO_ON:  aimotor_motors[s->side][s->axis].enabled = confirmed; break;
    case STEP_AI_SERVO_OFF: aimotor_motors[s->side][s->axis].enabled = !confirmed; break;
    case STEP_MW_RUN:       mw_motors[s->side][s->axis].enabled = confirmed; break;
    case STEP_MW_CLOSE:     mw_motors[s->side][s->axis].enabled = !confirmed; break;
    default: break;
    }
}

/* 清除当前步骤对应轴的 fault（回包确认成功后调用） */
static void CtrlSeqClearAxisFault(void)
{
    const SeqStep_t *s = &g_ctrl_seq.steps[g_ctrl_seq.step];
    /* 轴故障只由成功 ENABLE（ServoOn/0x88）清除。STOP/CLOSE/ServoOff
       成功只能证明安全命令已确认，不能抹掉此前的运动通信故障。 */
    if (s->op == STEP_MW_RUN) {
        mw_motors[s->side][s->axis].fault = 0;
        mw_motors[s->side][s->axis].retry_count = 0;
        mw_motors[s->side][s->axis].step = MW_STEP_IDLE;
        if (s->axis >= 1U) mw_coupled[s->side].fault = 0;
    } else if (s->op == STEP_AI_SERVO_ON) {
        aimotor_motors[s->side][s->axis].fault = 0;
        aimotor_motors[s->side][s->axis].retry_count = 0;
        aimotor_motors[s->side][s->axis].step = MOTOR_STEP_IDLE;
    }
}

/* 控制序列单步推进：发送命令 / 等待回包 / 超时重试 / 失败后继续下一台。
   每 5ms tick 最多发送一条命令；不阻塞主循环（无 HAL_Delay）。
   任一轴失败只记 failed_mask，继续处理其余轴（STOP/DISABLE 不因单轴失败放弃）。
   DRY_RUN：发送函数被短路为 mock 记录，序列立即确认，不等待回包。 */
static void CtrlSeqTick(void)
{
    if (g_ctrl_seq.kind == CTRL_SEQ_NONE) return;

    const SeqStep_t *s = &g_ctrl_seq.steps[g_ctrl_seq.step];
    uint32_t now = HAL_GetTick();

    if (g_ctrl_seq.phase == 0) {
        /* 发送当前步骤命令（DRY_RUN 下由发送函数内部 mock 记录） */
        CtrlSeqSendCmd();
        g_ctrl_seq.tx_tick = now;
#if AIMOTOR_DRY_RUN
#if AIMOTOR_SELF_TEST
        if (g_test_reply_wait) {
            g_ctrl_seq.phase = 1;   /* 自检：强制等待回包，用于重试/超时测试 */
        } else
#endif
        {
            /* DRY_RUN：无真实总线，回包永不到达；直接确认并使能状态按命令更新 */
            switch (s->op) {
            case STEP_AI_SERVO_ON:  aimotor_motors[s->side][s->axis].enabled = 1; break;
            case STEP_AI_SERVO_OFF: aimotor_motors[s->side][s->axis].enabled = 0; break;
            case STEP_MW_RUN:       mw_motors[s->side][s->axis].enabled = 1; break;
            case STEP_MW_CLOSE:     mw_motors[s->side][s->axis].enabled = 0; break;
            default: break;
            }
            CtrlSeqClearAxisFault();   /* 与真实合法回包路径保持相同故障恢复语义 */
            g_ctrl_seq.step++;
            g_ctrl_seq.phase = 0;
            g_ctrl_seq.retry = 0;
            if (g_ctrl_seq.step >= g_ctrl_seq.nsteps) {
                CtrlSeqFinish();
            }
        }
#else
        g_ctrl_seq.phase = 1;
#endif
        return;
    }

    /* phase == 1：等待回包 */
    if (CtrlSeqStepReplyOk()) {
        /* 该轴确认成功：更新实际使能状态并清除该轴 fault */
        CtrlSeqSetAxisEnabled(1);
        CtrlSeqClearAxisFault();
        g_ctrl_seq.step++;
        g_ctrl_seq.phase = 0;
        g_ctrl_seq.retry = 0;
        if (g_ctrl_seq.step >= g_ctrl_seq.nsteps) {
            CtrlSeqFinish();
        }
        return;
    }

    if ((now - g_ctrl_seq.tx_tick) < AIMOTOR_RESPONSE_TIMEOUT_MS) {
        return;   /* 等待应答中 */
    }
    /* 超时：重试或标记失败并继续下一台（重试计数只在重新发送时递增） */
    if (++g_ctrl_seq.retry <= CTRL_RETRY_MAX) {
        g_ctrl_seq.phase = 0;   /* 重新发送同一命令 */
        return;
    }
    /* 重试耗尽：该轴失败，记录并继续其余轴；所有失败类型都置对应轴 fault */
    uint16_t bit = AIMOTOR_AXIS_BIT(s->side, (s->op == STEP_MW_RUN ||
                                              s->op == STEP_MW_CLOSE) ? 3U + s->axis : s->axis);
    g_ctrl_seq.failed_mask |= bit;
    if (s->op == STEP_MW_RUN || s->op == STEP_MW_CLOSE) {
        mw_motors[s->side][s->axis].fault = 1;
    } else {
        aimotor_motors[s->side][s->axis].fault = 1;   /* 含 SERVO_ON/OFF 失败 */
    }
    g_ctrl_seq.step++;
    g_ctrl_seq.phase = 0;
    g_ctrl_seq.retry = 0;
    if (g_ctrl_seq.step >= g_ctrl_seq.nsteps) {
        CtrlSeqFinish();
    }
}

/* 控制序列是否进行中（mwmotor.c 等模块用其门控状态机） */
uint8_t Aimotor_CtrlSeqActive(void)
{
    return g_ctrl_seq.kind != CTRL_SEQ_NONE;
}

/* 文本 STOP 入口：异步全局安全停止（非阻塞、回包确认），供文本命令使用 */
void Aimotor_ControlStopStart(void)
{
    CtrlSeqStart(CTRL_SEQ_STOP, 0, 0, 0);
}

static uint8_t Aimotor_RequestTimedOut(const Aimotor_t *motor)
{
    return (uint8_t)((HAL_GetTick() - motor->last_tx_tick) >=
                     AIMOTOR_RESPONSE_TIMEOUT_MS);
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

    /* 等待停止位完全送出 */
    uint32_t start = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start) >= AIMOTOR_DMA_TIMEOUT) {
            break;
        }
    }

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

/* ======================================================================== */
/*                       回包解析处理                                       */
/* ======================================================================== */

static uint8_t ProcessRx(uint8_t bus_idx, uint8_t motor_idx)
{
    Aimotor_t *motor = &aimotor_motors[bus_idx][motor_idx];
    uint8_t ok = 0;

    motor->rx_ready = 0;

    /* 先检查长度，避免 rx_len - 2 下溢 */
    if (motor->rx_len < 5U) {
        return 0;
    }

    /* CRC 校验 */
    uint16_t calc_crc = Aimotor_CRC16(motor->rx_buf, motor->rx_len - 2);
    uint16_t rec_crc  = (motor->rx_buf[motor->rx_len - 1] << 8)
                      |  motor->rx_buf[motor->rx_len - 2];
    if (calc_crc != rec_crc) {
        return 0;
    }

    if (motor->rx_buf[0] != motor->slave_id) {
        return 0;
    }

    switch (motor->step) {
    case MOTOR_STEP_WAIT_STOP:
        ok = (motor->rx_len == 8U && motor->rx_buf[1] == 0x10U &&
              motor->rx_buf[2] == 0x03U && motor->rx_buf[3] == 0x05U &&
              motor->rx_buf[4] == 0x00U && motor->rx_buf[5] == 0x01U);
        break;
    case MOTOR_STEP_WAIT_WRITE:
        ok = (motor->rx_len == 8U && motor->rx_buf[1] == 0x10U &&
              motor->rx_buf[2] == 0x11U && motor->rx_buf[3] == 0x0CU &&
              motor->rx_buf[4] == 0x00U && motor->rx_buf[5] == 0x02U);
        break;
    case MOTOR_STEP_WAIT_TRIGGER:
        ok = (motor->rx_len == 8U && motor->rx_buf[1] == 0x10U &&
              motor->rx_buf[2] == 0x03U && motor->rx_buf[3] == 0x05U &&
              motor->rx_buf[4] == 0x00U && motor->rx_buf[5] == 0x01U);
        break;
    case MOTOR_STEP_WAIT_QUERY:
        /* H0B_07绝对位置 */
        if (motor->rx_len == 9U && motor->rx_buf[1] == 0x03U &&
            motor->rx_buf[2] == 0x04U) {
            motor->actual_position =
                Aimotor_DecodeInt32CDAB(&motor->rx_buf[3]);
            motor->actual_valid = 1;
            ok = 1;
        }
        break;
    case MOTOR_STEP_WAIT_ERROR:
        /* H0B_15编码器位置偏差 */
        if (motor->rx_len == 9U && motor->rx_buf[1] == 0x03U &&
            motor->rx_buf[2] == 0x04U) {
            motor->position_error =
                Aimotor_DecodeInt32CDAB(&motor->rx_buf[3]);
            ok = 1;
        }
        break;
    default:
        break;
    }

    if (ok) {
        motor->retry_count = 0;
    }
    return ok;
}

/* ======================================================================== */
/*                       上位机命令解析                                     */
/* ======================================================================== */

/**
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
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        return 0;
    }

    cmd->motor_idx = (uint8_t)(m_ptr[1] - '1');
    if (cmd->motor_idx >= AIMOTOR_MOTORS_PER) {
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR INVALID MOTOR\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
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
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        return 0;
    }

    int32_t pos;
    if (!ParsePosition(p_ptr + 1, &pos)) {
        /* ParsePosition 失败：可能是格式错误或超范围 */
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR INVALID POSITION\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        return 0;
    }

    /* 范围二次确认（ParsePosition 内已检查，此处保底） */
    if (pos < AIMOTOR_POSITION_MIN || pos > AIMOTOR_POSITION_MAX) {
        int n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR POSITION OUT OF RANGE\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
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

static void HostCmd_Execute(const Aimotor_Cmd_t *cmd)
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
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        break;

    case AIMOTOR_CMD_TEST:
        /* 旧 B TEST 会绕过统一控制序列直接 Servo On，永久关闭该危险入口。 */
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR B TEST DISABLED: use binary ENABLE\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        break;

    case AIMOTOR_CMD_ENABLE:
        /* 单轴文本使能不再支持：无法在文本路径完成回包确认，直接写 enabled=1
           会与 STATE 的“实际确认使能”语义冲突。统一使用二进制 ENABLE 或
           ALL ENABLE（异步序列 + 回包确认）完成使能。 */
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR: single-axis EN disabled, use ALL ENABLE\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        break;

    case AIMOTOR_CMD_QUERY:
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "POS B%d M%d %ld\r\n",
                     b + 1, m + 1,
                     (long)aimotor_motors[b][m].actual_position);
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        break;

    case AIMOTOR_CMD_GO:
#if AIMOTOR_TEXT_MOTION_ENABLED
        if (g_control_state != CONTROL_ENABLED) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL DENIED\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        if (aimotor_motors[b][m].fault) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        /* 统一交给生产状态机 STOP→WRITE→TRIGGER→QUERY，等待并校验每个回包；
           不再使用累计 100ms 的 HAL_Delay 阻塞主循环。 */
        aimotor_motors[b][m].cmd_pending = 1;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK B%d M%d GO: ACCEPTED\r\n", b + 1, m + 1);
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
#else
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR TEXT MOTION DISABLED\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
#endif
        break;

    case AIMOTOR_CMD_HOME:
#if AIMOTOR_TEXT_MOTION_ENABLED
        if (g_control_state != CONTROL_ENABLED) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL DENIED\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        if (aimotor_motors[b][m].fault) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        aimotor_motors[b][m].target_position = 0;
        aimotor_motors[b][m].cmd_pending = 1;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK B%d M%d HOME\r\n", b + 1, m + 1);
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
#else
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR TEXT MOTION DISABLED\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
#endif
        break;

    case AIMOTOR_CMD_MOVE:
#if AIMOTOR_TEXT_MOTION_ENABLED
        if (g_control_state != CONTROL_ENABLED) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL DENIED\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        b = cmd->bus_idx;
        m = cmd->motor_idx;
        if (aimotor_motors[b][m].fault) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR AXIS FAULT: STOP THEN ENABLE\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        aimotor_motors[b][m].target_position = cmd->position;
        aimotor_motors[b][m].cmd_pending = 1;
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK B%d M%d P%ld: ACCEPTED\r\n",
                     b + 1, m + 1, (long)cmd->position);
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
#else
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "ERR TEXT MOTION DISABLED\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
#endif
        break;

    case AIMOTOR_CMD_ENABLE_ALL:
        if (Aimotor_CtrlSeqActive()) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL BUSY\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
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
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        if ((g_control_faults & CONTROL_FAULT_INTERNAL) != 0U) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL FAULT\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        CtrlSeqStart(CTRL_SEQ_ENABLE, 0, 0, 0);
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK ALL ENABLE: ACCEPTED\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        break;

    case AIMOTOR_CMD_DISABLE_ALL:
        if (Aimotor_CtrlSeqActive()) {
            n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                         "ERR CONTROL BUSY\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
            break;
        }
        CtrlSeqStart(CTRL_SEQ_DISABLE, 0, 0, 0);
        n = snprintf(g_host_tx_buf, sizeof(g_host_tx_buf),
                     "OK ALL DISABLE: ACCEPTED\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)g_host_tx_buf, n, 100);
        break;

    case AIMOTOR_CMD_NONE:
    default:
        break;
    }
}

/* ======================================================================== */
/*                         系统初始化                                       */
/* ======================================================================== */

void Aimotor_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    uint8_t b, m;

    /* ── PB0 LED 初始化：总线 #1 (USART2) TX 指示 ────────────────── */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Pin = GPIO_PIN_0;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    /* ── PB2 LED 初始化：总线 #2 (USART3) TX 指示 ────────────────── */
    gpio_init.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    /* ── 初始化总线 #0: USART2 (PA2/PA3), PB0 LED ───────────────── */
    aimotor_buses[0].huart    = &huart2;
    aimotor_buses[0].led_port = GPIOB;
    aimotor_buses[0].led_pin  = GPIO_PIN_0;
    aimotor_buses[0].rx_ready = 0;
    aimotor_buses[0].rx_len   = 0;
    aimotor_buses[0].current_motor = 0;

    /* ── 初始化总线 #1: USART3 (PB10/PB11), PB2 LED ─────────────── */
    aimotor_buses[1].huart    = &huart3;
    aimotor_buses[1].led_port = GPIOB;
    aimotor_buses[1].led_pin  = GPIO_PIN_2;
    aimotor_buses[1].rx_ready = 0;
    aimotor_buses[1].rx_len   = 0;
    aimotor_buses[1].current_motor = 0;

    /* ── 初始化每总线的 3 个电机 ────────────────────────────────── */
    for (b = 0; b < AIMOTOR_BUS_COUNT; b++) {
        for (m = 0; m < AIMOTOR_MOTORS_PER; m++) {
            aimotor_motors[b][m].slave_id       = m + 1;
            aimotor_motors[b][m].target_position = 0;
            aimotor_motors[b][m].actual_position = 0;
            aimotor_motors[b][m].position_error = 0;
            aimotor_motors[b][m].step            = MOTOR_STEP_IDLE;
            aimotor_motors[b][m].retry_count     = 0;
            aimotor_motors[b][m].cmd_pending     = 0;
            aimotor_motors[b][m].enabled         = 0;
            aimotor_motors[b][m].arrived         = 0;
            aimotor_motors[b][m].arrive_count    = 0;
            aimotor_motors[b][m].fault           = 0;
            aimotor_motors[b][m].last_tx_tick    = 0;
            aimotor_motors[b][m].move_start_tick = HAL_GetTick();
            aimotor_motors[b][m].actual_valid    = 0;
        }
    }

    /* ── 启动各 RS485 总线的 DMA IDLE 接收 ───────────────────────── */
    HAL_UARTEx_ReceiveToIdle_DMA(aimotor_buses[0].huart,
                                  aimotor_buses[0].rx_buf, AIMOTOR_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(aimotor_buses[1].huart,
                                  aimotor_buses[1].rx_buf, AIMOTOR_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(aimotor_buses[0].huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(aimotor_buses[1].huart->hdmarx, DMA_IT_HT);

    /* ── 上电安全停止序列（异步，不阻塞） ──────────────────────────
       复位后向全部 AI 电机发送真实 Servo Off（0x0303=0），向全部 MW 电机
       发送 0x80 关闭，清除全部待执行目标；序列完成前控制状态保持
       DISABLED，完成后仍为 DISABLED。绝不进入 ENABLED，绝不自动恢复
       复位前目标。序列由主循环 CtrlSeqTick 推进。 */
    g_ctrl_seq.kind = CTRL_SEQ_NONE;
    g_ctrl_seq.active = 0;
    CtrlSeqStart(CTRL_SEQ_STARTUP_SAFE_STOP, 0, 0, 0);
}

/* ======================================================================== */
/*                        DMA IDLE 回调路由                                */
/* ======================================================================== */

void Aimotor_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t i, m;

    /* Size 为 0 时重新启动 DMA 后直接返回 */
    if (Size == 0U) {
        for (i = 0; i < AIMOTOR_BUS_COUNT; i++) {
            if (aimotor_buses[i].huart == huart) {
                HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                              aimotor_buses[i].rx_buf,
                                              AIMOTOR_BUF_SIZE);
                __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
                return;
            }
        }
        return;
    }

    /* 按从站号路由到对应电机私有缓冲 */
    for (i = 0; i < AIMOTOR_BUS_COUNT; i++) {
        if (aimotor_buses[i].huart == huart) {
            uint8_t slave = aimotor_buses[i].rx_buf[0];
            for (m = 0; m < AIMOTOR_MOTORS_PER; m++) {
                if (aimotor_motors[i][m].slave_id == slave) {
                    uint16_t copy_len = Size;
                    if (copy_len > sizeof(aimotor_motors[i][m].rx_buf)) {
                        copy_len = sizeof(aimotor_motors[i][m].rx_buf);
                    }

                    memcpy(aimotor_motors[i][m].rx_buf,
                           aimotor_buses[i].rx_buf,
                           copy_len);

                    /* rx_len 字段为 uint8_t，本处最大 16，转换安全 */
                    aimotor_motors[i][m].rx_len = (uint8_t)copy_len;
                    aimotor_motors[i][m].rx_ready = 1;
                    break;
                }
            }
            HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                          aimotor_buses[i].rx_buf,
                                          AIMOTOR_BUF_SIZE);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

void Aimotor_RecoverRx(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < AIMOTOR_BUS_COUNT; ++i) {
        if (aimotor_buses[i].huart == huart) {
            HAL_UART_AbortReceive(huart);
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            aimotor_buses[i].rx_ready = 0;
            aimotor_buses[i].rx_len = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                          aimotor_buses[i].rx_buf,
                                          AIMOTOR_BUF_SIZE);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

/* ======================================================================== */
/*                        主状态机调度                                     */
/* ======================================================================== */

void Aimotor_Process(void)
{
    static uint8_t round_robin = 0;   /* 轮转索引 0/1/2 */
    uint8_t b;
    Aimotor_t *motor;

    /* 上位机二进制流解析（拆包/粘包/CRC 重同步 + 文本行提取） */
    Aimotor_HostStreamPoll();

    /* 异步控制序列推进（ENABLE/STOP/DISABLE/上电安全停止）。
       序列进行期间暂停 AI 轮转与 MW 状态机，序列独占两族总线，
       避免同一条 RS485 总线上同时存在两个未完成请求。 */
    CtrlSeqTick();

    /* ── 处理上位机命令（非阻塞轮询, 只消费 B/ALL 命令） ──────────
       控制序列活动时只允许文本 STOP 穿透并抢占；其它文本命令继续排队。 */
    if (g_host_cmd_ready) {
        /* 判断首字符: B→AI电机, A→ALL, L/R→MW电机(不消费) */
        uint8_t c = g_host_cmd_buf[0];
        if (c == 'B' || c == 'b' || c == 'A' || c == 'a') {
            uint8_t is_stop = (strstr(g_host_cmd_buf, "STOP") != NULL ||
                               strstr(g_host_cmd_buf, "stop") != NULL);
            if (!Aimotor_CtrlSeqActive() || is_stop) {
                g_host_cmd_ready = 0;
                Aimotor_Cmd_t cmd;
                if (HostCmd_Parse((const uint8_t *)g_host_cmd_buf,
                                  g_host_cmd_len, &cmd)) {
                    HostCmd_Execute(&cmd);
                }
            }
        }
    }

    if (Aimotor_CtrlSeqActive()) {
        return;   /* 序列进行中：暂停 AI/MW 状态机，不接收新目标 */
    }

    /* ── 轮转调度：每个tick每条总线处理一个电机 ─────────────────── */
    for (b = 0; b < AIMOTOR_BUS_COUNT; b++) {
        /* 总线唯一事务 owner：若该总线存在未完成请求（任一电机处于 WAIT_*），
           本轮只处理该电机（回包+超时），不启动其它电机的新事务；
           保证一条 RS485 总线上同一时刻只有一个未完成请求，回包不会
           因轮转跳过而滞留或产生总线命令重叠。 */
        uint8_t owner = 0xFF;
        for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
            Aimotor_Step_t st = aimotor_motors[b][m].step;
            if (st == MOTOR_STEP_WAIT_STOP || st == MOTOR_STEP_WAIT_WRITE ||
                st == MOTOR_STEP_WAIT_TRIGGER || st == MOTOR_STEP_WAIT_QUERY ||
                st == MOTOR_STEP_WAIT_ERROR) {
                owner = m;
                break;
            }
        }
        motor = (owner != 0xFF) ? &aimotor_motors[b][owner]
                                : &aimotor_motors[b][round_robin];
        uint8_t mi = (uint8_t)(motor - aimotor_motors[b]);

        /* 只在没有未完成请求时接收新目标，避免打断一个Modbus事务。 */
        if (motor->cmd_pending &&
            (motor->step == MOTOR_STEP_IDLE ||
             motor->step == MOTOR_STEP_ARRIVED) &&
            !motor->fault) {
            motor->cmd_pending = 0;
            motor->retry_count = 0;
            motor->arrived = 0;
            motor->arrive_count = 0;
            motor->actual_valid = 0;   /* 新目标执行期间反馈视为非最新 */
            motor->move_start_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_STOP;
        }

        /* 处理当前请求的回包；无效回包会进入超时重试路径。 */
        if (motor->rx_ready &&
            (motor->step == MOTOR_STEP_WAIT_STOP ||
             motor->step == MOTOR_STEP_WAIT_WRITE ||
             motor->step == MOTOR_STEP_WAIT_TRIGGER ||
             motor->step == MOTOR_STEP_WAIT_QUERY ||
             motor->step == MOTOR_STEP_WAIT_ERROR)) {
            if (ProcessRx(b, mi)) {
                switch (motor->step) {
                case MOTOR_STEP_WAIT_STOP:
                    motor->step = MOTOR_STEP_WRITE;
                    break;
                case MOTOR_STEP_WAIT_WRITE:
                    motor->step = MOTOR_STEP_TRIGGER;
                    break;
                case MOTOR_STEP_WAIT_TRIGGER:
                    motor->step = MOTOR_STEP_QUERY;
                    break;
                case MOTOR_STEP_WAIT_QUERY:
#if AIMOTOR_ENABLE_ARRIVAL_CHECK
                    motor->step = MOTOR_STEP_QUERY_ERROR;
#else
                    /* 到位检测暂时注释：先验证位置读写和通信稳定性。 */
                    motor->step = MOTOR_STEP_IDLE;
#endif
                    break;
                case MOTOR_STEP_WAIT_ERROR:
#if AIMOTOR_ENABLE_ARRIVAL_CHECK
                    if (motor->position_error <=
                            AIMOTOR_POS_ERROR_TOLERANCE_ENCODER &&
                        motor->position_error >=
                            -AIMOTOR_POS_ERROR_TOLERANCE_ENCODER) {
                        if (motor->arrive_count < AIMOTOR_ARRIVE_SAMPLES)
                            motor->arrive_count++;
                    } else {
                        motor->arrive_count = 0;
                    }
                    motor->arrived =
                        (motor->arrive_count >= AIMOTOR_ARRIVE_SAMPLES);
                    if (motor->cmd_pending) {
                        motor->step = MOTOR_STEP_STOP;
                    } else if (motor->arrived) {
                        motor->step = MOTOR_STEP_ARRIVED;
                    } else if ((HAL_GetTick() - motor->move_start_tick) >=
                               AIMOTOR_MOVE_TIMEOUT_MS) {
                        motor->fault = 1;
                        motor->step = MOTOR_STEP_FAULT;
                    } else {
                        motor->step = MOTOR_STEP_QUERY;
                    }
#else
                    /* 到位检测暂时注释，保留 H0B-15 回包解析供后续启用。 */
                    motor->step = MOTOR_STEP_IDLE;
#endif
                    break;
                default:
                    break;
                }
            }
        }

        switch (motor->step) {

        case MOTOR_STEP_IDLE:
            /* 空闲时保持位置和位置偏差缓存最新。 */
            motor->step = MOTOR_STEP_QUERY;
            break;

        case MOTOR_STEP_STOP:
            motor->rx_ready = 0;
            Aimotor_StopMotion(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            /* 重试计数只在事务启动时清零；重发时保留，确保达到上限后停发 */
            motor->step = MOTOR_STEP_WAIT_STOP;
            break;

        case MOTOR_STEP_WRITE:
            motor->rx_ready = 0;
            Aimotor_SendPosition(b, motor->slave_id,
                                 motor->target_position);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_WRITE;
            break;

        case MOTOR_STEP_TRIGGER:
            motor->rx_ready = 0;
            Aimotor_TriggerMotion(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_TRIGGER;
            break;

        case MOTOR_STEP_QUERY:
            motor->rx_ready = 0;
            Aimotor_QueryPosition(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_QUERY;
            break;

        case MOTOR_STEP_QUERY_ERROR:
            motor->rx_ready = 0;
            Aimotor_QueryPositionError(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_ERROR;
            break;

        case MOTOR_STEP_WAIT_STOP:
        case MOTOR_STEP_WAIT_WRITE:
        case MOTOR_STEP_WAIT_TRIGGER:
        case MOTOR_STEP_WAIT_QUERY:
        case MOTOR_STEP_WAIT_ERROR:
            if (Aimotor_RequestTimedOut(motor)) {
                motor->rx_ready = 0;
                if (++motor->retry_count > AIMOTOR_RETRY_MAX) {
                    motor->retry_count = 0;
                    motor->fault = 1;
                    motor->step = MOTOR_STEP_FAULT;
                } else {
                    switch (motor->step) {
                    case MOTOR_STEP_WAIT_STOP:    motor->step = MOTOR_STEP_STOP; break;
                    case MOTOR_STEP_WAIT_WRITE:   motor->step = MOTOR_STEP_WRITE; break;
                    case MOTOR_STEP_WAIT_TRIGGER: motor->step = MOTOR_STEP_TRIGGER; break;
                    case MOTOR_STEP_WAIT_QUERY:   motor->step = MOTOR_STEP_QUERY; break;
                    case MOTOR_STEP_WAIT_ERROR:   motor->step = MOTOR_STEP_QUERY_ERROR; break;
                    default: break;
                    }
                }
            }
            break;

        case MOTOR_STEP_ARRIVED:
            if (motor->cmd_pending) {
                motor->step = MOTOR_STEP_STOP;
            }
            break;

        case MOTOR_STEP_FAULT:
            /* 故障锁存：TARGET 校验会拒绝故障轴；仅成功 ENABLE 清除并恢复 IDLE。 */
            motor->cmd_pending = 0;
            break;

        default:
            motor->step = MOTOR_STEP_FAULT;
            motor->fault = 1;
            break;
        }
    }

    /* 下一轮转索引 */
    round_robin = (round_robin + 1) % AIMOTOR_MOTORS_PER;
}

/* ======================================================================== */
/*            固件自检（AIMOTOR_SELF_TEST，直接调用生产 C 函数）                */
/* ======================================================================== */
#if AIMOTOR_SELF_TEST

static void STestOut(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void STestReport(const char *name, int cond)
{
    char buf[80];
    int n = snprintf(buf, sizeof(buf), "  [%s] %s\r\n",
                     cond ? "PASS" : "FAIL", name);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
    if (cond) g_test_pass++; else g_test_fail++;
}

/* 构造合法帧：AA 55 | VER | CMD | SEQ(2) | LEN(2) | PAYLOAD | CRC(2) */
static uint16_t STestBuildFrame(uint8_t *out, uint8_t cmd, uint16_t seq,
                                const uint8_t *payload, uint16_t plen)
{
    out[0] = 0xAA; out[1] = 0x55; out[2] = 0x01; out[3] = cmd;
    out[4] = (uint8_t)(seq & 0xFF); out[5] = (uint8_t)(seq >> 8);
    out[6] = (uint8_t)(plen & 0xFF); out[7] = (uint8_t)(plen >> 8);
    for (uint16_t i = 0; i < plen; i++) out[8 + i] = payload[i];
    uint16_t crc = Aimotor_CRC16(&out[2], (uint16_t)(6U + plen));
    out[8 + plen] = (uint8_t)(crc & 0xFF);
    out[9 + plen] = (uint8_t)(crc >> 8);
    return (uint16_t)(10U + plen);
}

/* 合法 TARGET 帧（左臂，J1..J3=100/200/300mm，J4..J6=20/20/45°） */
static uint16_t STestBuildTarget(uint8_t *out, uint16_t seq)
{
    uint8_t payload[28];
    memset(payload, 0, sizeof(payload));
    payload[0] = 0;   /* arm=左 */
    const double urad_per_deg = 1e6 * 3.14159265358979323846 / 180.0;
    int32_t joints[6] = { 100000, 200000, 300000,
                          (int32_t)(20.0 * urad_per_deg),
                          (int32_t)(20.0 * urad_per_deg),
                          (int32_t)(45.0 * urad_per_deg) };
    for (int i = 0; i < 6; i++) {
        uint8_t *p = &payload[4 + i * 4];
        p[0] = (uint8_t)(joints[i] & 0xFF);
        p[1] = (uint8_t)((joints[i] >> 8) & 0xFF);
        p[2] = (uint8_t)((joints[i] >> 16) & 0xFF);
        p[3] = (uint8_t)((joints[i] >> 24) & 0xFF);
    }
    return STestBuildFrame(out, 0x10, seq, payload, sizeof(payload));
}

/* 清空环形缓冲与捕获缓冲（每个子测试前调用）；故障位清零保证子测试独立 */
static void STestReset(void)
{
    g_host_ring_head = 0;
    g_host_ring_tail = 0;
    g_host_text_len = 0;
    g_host_cmd_ready = 0;
    g_host_rx_ready = 0;
    g_test_tx_len = 0;
    g_test_reply_wait = 0;
    g_control_faults = CONTROL_FAULT_NONE;
}

static void STestSeqTraceReset(void)
{
    g_test_seq_send_count = 0;
    g_test_seq_ai_stop_mask = 0;
    g_test_seq_ai_off_mask = 0;
    g_test_seq_mw_close_mask = 0;
}

/* 统计捕获中的指定响应帧数量（0x80 ACK / 0x81 STATE） */
static int STestCountFrames(uint8_t want_cmd)
{
    int n = 0;
    for (uint16_t i = 0; i + 4U < g_test_tx_len; i++) {
        if (g_test_tx_log[i] == 0xAA && g_test_tx_log[i + 1] == 0x55 &&
            g_test_tx_log[i + 2] == 0x01 && g_test_tx_log[i + 3] == want_cmd) {
            n++;
        }
    }
    return n;
}

/* 最近一个 ACK 的 result 字节 */
static uint8_t STestLastAckResult(void)
{
    for (int i = (int)g_test_tx_len - 14; i >= 0; i--) {
        if (g_test_tx_log[i] == 0xAA && g_test_tx_log[i + 1] == 0x55 &&
            g_test_tx_log[i + 2] == 0x01 && g_test_tx_log[i + 3] == 0x80) {
            return g_test_tx_log[i + 11];
        }
    }
    return 0xFF;
}

/* 推进控制序列直到完成（DRY_RUN 下每步立即确认） */
static void STestRunSeq(void)
{
    for (int i = 0; i < 200 && Aimotor_CtrlSeqActive(); i++) {
        CtrlSeqTick();
    }
}

/* ── 解析器进度保证（P0） ──────────────────────────────────────── */
static void STestParser(void)
{
    uint8_t f[80];
    STestOut("\n== 解析器（生产 Aimotor_HostStreamPoll / Aimotor_HandleBinaryFrame）==\r\n");

    /* 1) 非法 LEN（>MAX）后跟合法帧：函数返回、合法帧被解析 */
    STestReset();
    uint8_t badlen[8] = {0xAA, 0x55, 0x01, 0x10, 0x01, 0x00, 0x00, 0x01}; /* LEN=256 */
    g_test_capture_tx = 1;
    uint16_t tail_before = g_host_ring_tail;
    Aimotor_HostRxAppend(badlen, sizeof(badlen));
    Aimotor_HostStreamPoll();
    STestReport("LEN>MAX 后 poll 能返回且消费非法帧头",
                g_host_ring_tail != tail_before);
    uint16_t n0 = STestBuildTarget(f, 0x10);
    STestReset();
    Aimotor_HostRxAppend(badlen, sizeof(badlen));
    Aimotor_HostRxAppend(f, n0);
    Aimotor_HostStreamPoll();
    STestReport("非法LEN后跟合法帧 → 合法帧被解析（ACK 出现）",
                STestCountFrames(0x80) == 1);

    /* 2) CRC 错误帧后跟合法帧（DISABLED 状态下合法 TARGET 返回 0x06） */
    STestReset();
    uint8_t f1[80], f2[80];
    uint16_t n1 = STestBuildTarget(f1, 0x11);
    f1[n1 - 1] ^= 0xFF;   /* 破坏 CRC（独立缓冲，避免覆盖合法帧） */
    uint16_t n2 = STestBuildTarget(f2, 0x12);
    uint8_t stream[80];
    memcpy(stream, f1, n1); memcpy(stream + n1, f2, n2);
    Aimotor_HostRxAppend(stream, (uint16_t)(n1 + n2));
    Aimotor_HostStreamPoll();
    STestReport("CRC错误帧后跟合法帧 → CRC ACK + 合法 ACK",
                STestCountFrames(0x80) == 2 && STestLastAckResult() == 0x06);

    /* 3) 噪声中含单独 AA；AA AA 55 能同步 */
    STestReset();
    uint8_t noise[8] = {0x41, 0xAA, 0x41, 0xAA, 0xAA, 0x41, 0x41, 0x41};
    uint16_t n3 = STestBuildTarget(f, 0x13);
    uint8_t stream2[88];
    memcpy(stream2, noise, 8); memcpy(stream2 + 8, f, n3);
    Aimotor_HostRxAppend(stream2, (uint16_t)(8U + n3));
    Aimotor_HostStreamPoll();
    STestReport("噪声+AA+AA+AA55 重同步 → 合法帧解析", STestCountFrames(0x80) == 1);

    /* 4) 半帧等待：前 10 字节后无事件，补足后解析 */
    STestReset();
    uint16_t n4 = STestBuildTarget(f, 0x14);
    Aimotor_HostRxAppend(f, 10);
    Aimotor_HostStreamPoll();
    STestReport("半帧（前10字节）→ 无事件", STestCountFrames(0x80) == 0);
    Aimotor_HostRxAppend(&f[10], (uint16_t)(n4 - 10U));
    Aimotor_HostStreamPoll();
    STestReport("补齐后续字节 → 解析成功", STestCountFrames(0x80) == 1);

    /* 5) 两帧/三帧粘包 */
    STestReset();
    uint16_t n5 = STestBuildTarget(f, 0x15);
    uint8_t s3[120];
    memcpy(s3, f, n5); memcpy(s3 + n5, f, n5);
    Aimotor_HostRxAppend(s3, (uint16_t)(2U * n5));
    Aimotor_HostStreamPoll();
    STestReport("两帧粘包 → 2 个 ACK", STestCountFrames(0x80) == 2);
    STestReset();
    memcpy(s3, f, n5); memcpy(s3 + n5, f, n5); memcpy(s3 + 2U * n5, f, n5);
    Aimotor_HostRxAppend(s3, (uint16_t)(3U * n5));
    Aimotor_HostStreamPoll();
    STestReport("三帧粘包 → 3 个 ACK", STestCountFrames(0x80) == 3);

    /* 6) 缓冲溢出（>256 字节）后恢复：环形缓冲满置 RX_OVERFLOW，
          解析器排空后，后续合法帧仍能解析（模拟真实 DMA 与主循环交错）。 */
    STestReset();
    uint8_t flood[300];
    memset(flood, 0xAA, sizeof(flood));
    Aimotor_HostRxAppend(flood, sizeof(flood));   /* 环形缓冲填满 → 溢出 */
    Aimotor_HostStreamPoll();                      /* 排空（AA AA 噪声被丢弃） */
    STestReport("缓冲溢出置 RX_OVERFLOW 故障位",
                (g_control_faults & CONTROL_FAULT_RX_OVERFLOW) != 0U);
    uint16_t n6 = STestBuildTarget(f, 0x16);
    Aimotor_HostRxAppend(f, n6);
    Aimotor_HostStreamPoll();
    STestReport("溢出排空后合法帧仍可解析", STestCountFrames(0x80) == 1);

    /* 7) 随机输入 fuzz：多轮随机字节 + 穿插合法帧，poll 不得卡死 */
    {
        int parsed = 0, completed = 0, rounds = 100;
        uint8_t ring_bounded = 1;
        uint32_t seed = 0x5A5A5A5A;
        for (int r = 0; r < rounds; r++) {
            STestReset();
            uint8_t buf[200];
            for (int i = 0; i < 200; i++) {
                seed = seed * 1664525U + 1013904223U;
                buf[i] = (uint8_t)(seed >> 24);
            }
            Aimotor_HostRxAppend(buf, sizeof(buf));
            if (r % 3 == 0) {
                uint16_t n7 = STestBuildTarget(f, (uint16_t)(0x20 + r));
                Aimotor_HostRxAppend(f, n7);
            }
            Aimotor_HostStreamPoll();   /* 必须返回 */
            completed++;
            uint16_t used = (uint16_t)((g_host_ring_head + PROTOCOL_RX_BUFFER_SIZE -
                                        g_host_ring_tail) % PROTOCOL_RX_BUFFER_SIZE);
            if (used >= PROTOCOL_RX_BUFFER_SIZE) ring_bounded = 0;
            if (STestCountFrames(0x80) == 1) parsed++;
        }
        STestReport("随机输入 fuzz 100 轮均返回且环形缓冲保持有界",
                    completed == rounds && ring_bounded);
        STestReport("fuzz 穿插的合法帧被解析（>0）", parsed > 0);
    }

    /* 8) 每次解析循环的进度不变量由结构保证（消费或退出），
          非法 LEN 分支至少消费 1 字节：tail 前进验证 */
    {
        STestReset();
        uint16_t before = g_host_ring_tail;
        Aimotor_HostRxAppend(badlen, sizeof(badlen));
        Aimotor_HostStreamPoll();
        uint16_t advanced = (uint16_t)((g_host_ring_tail +
                                        PROTOCOL_RX_BUFFER_SIZE - before) %
                                       PROTOCOL_RX_BUFFER_SIZE);
        STestReport("非法LEN 分支 tail 前进（进度保证）",
                    advanced >= 1U);
    }
    g_test_capture_tx = 0;
}

/* ── 看门狗刷新表（P0） ───────────────────────────────────────── */
static void STestWatchdog(void)
{
    uint8_t f[80];
    STestOut("\n== 通信看门狗（生产 Aimotor_HandleBinaryFrame / CommWatchdog）==\r\n");

    /* 进入 ENABLED：通过真实 ENABLE 帧 + 序列 */
    STestReset();
    g_test_capture_tx = 1;
    STestBuildFrame(f, 0x02, 0x30, NULL, 0);   /* ENABLE */
    Aimotor_HostRxAppend(f, 10);
    Aimotor_HostStreamPoll();
    STestRunSeq();
    STestReport("ENABLE 序列完成 → 状态 ENABLED", Aimotor_GetControlState() == CONTROL_ENABLED);

    /* ENABLED 下重复 ENABLE 必须幂等：立即 ACK，不重跑 12 轴序列；随后断联
       仍由 250ms 看门狗停止，不存在控制序列屏蔽窗口。 */
    {
        STestReset();
        uint32_t stale = HAL_GetTick() - 500U;
        g_last_valid_frame_ms = stale;
        STestBuildFrame(f, 0x02, 0x30A, NULL, 0);
        Aimotor_HostRxAppend(f, 10);
        Aimotor_HostStreamPoll();
        STestReport("ENABLED 下重复 ENABLE → 幂等 ACK 且不启动控制序列",
                    STestLastAckResult() == AIMOTOR_ACK_OK &&
                    !Aimotor_CtrlSeqActive() &&
                    Aimotor_GetControlState() == CONTROL_ENABLED);
        STestReport("幂等 ENABLE 立即刷新看门狗", g_last_valid_frame_ms != stale);

        g_last_valid_frame_ms = HAL_GetTick() - (HOST_COMM_WATCHDOG_MS + 1U);
        Aimotor_CommWatchdog();
        STestReport("幂等 ENABLE 后断联 250ms → 看门狗仍启动全局 STOP",
                    Aimotor_GetControlState() == CONTROL_STOPPED &&
                    Aimotor_CtrlSeqActive());
        STestRunSeq();

        /* 恢复后续测试所需 ENABLED 状态。 */
        STestReset();
        STestBuildFrame(f, 0x02, 0x30B, NULL, 0);
        Aimotor_HostRxAppend(f, 10);
        Aimotor_HostStreamPoll();
        STestRunSeq();
        STestReport("看门狗 STOP 后重新 ENABLE → ENABLED",
                    Aimotor_GetControlState() == CONTROL_ENABLED);
    }

    /* 合法 TARGET 刷新 */
    {
        uint32_t before = g_last_valid_frame_ms;
        uint16_t nt = STestBuildTarget(f, 0x31);
        STestReset();
        Aimotor_HostRxAppend(f, nt);
        Aimotor_HostStreamPoll();
        STestReport("合法 TARGET → ACK_OK + 刷新看门狗",
                    STestLastAckResult() == 0x00 && g_last_valid_frame_ms >= before);
    }

    /* 以下“不刷新”测试先把时间戳标记为陈旧，若被刷新则必然改变 */
#define STEST_STALE_MARK() do { g_last_valid_frame_ms = HAL_GetTick() - 500U; } while (0)
#define STEST_STILL_STALE() (g_last_valid_frame_ms <= (HAL_GetTick() - 400U))

    /* CRC 错误不刷新 */
    {
        uint16_t nt = STestBuildTarget(f, 0x32);
        f[nt - 1] ^= 0xFF;
        STestReset();
        STEST_STALE_MARK();
        Aimotor_HostRxAppend(f, nt);
        Aimotor_HostStreamPoll();
        STestReport("CRC 错误帧不刷新看门狗", STEST_STILL_STALE());
    }

    /* 非法 LEN 不刷新 */
    {
        uint8_t badlen[8] = {0xAA, 0x55, 0x01, 0x10, 0x00, 0x00, 0x00, 0x01};
        STestReset();
        STEST_STALE_MARK();
        Aimotor_HostRxAppend(badlen, sizeof(badlen));
        Aimotor_HostStreamPoll();
        STestReport("非法 LEN 不刷新看门狗", STEST_STILL_STALE());
    }

    /* 未知 CMD 不刷新 */
    {
        STestReset();
        STEST_STALE_MARK();
        STestBuildFrame(f, 0x7F, 0x33, NULL, 0);
        Aimotor_HostRxAppend(f, 10);
        Aimotor_HostStreamPoll();
        STestReport("未知 CMD 不刷新看门狗",
                    STEST_STILL_STALE() && STestLastAckResult() == 0x03);
    }

    /* arm 错误（TARGET arm=2）不刷新 */
    {
        STestReset();
        STEST_STALE_MARK();
        STestBuildTarget(f, 0x34);
        f[8] = 2;   /* arm=2 */
        uint16_t len2 = (uint16_t)f[6];
        uint16_t crc2 = Aimotor_CRC16(&f[2], (uint16_t)(6U + len2));
        f[8 + len2] = (uint8_t)(crc2 & 0xFF);
        f[9 + len2] = (uint8_t)(crc2 >> 8);
        Aimotor_HostRxAppend(f, (uint16_t)(10U + len2));
        Aimotor_HostStreamPoll();
        STestReport("TARGET arm=2 → BAD_ARM + 不刷新",
                    STEST_STILL_STALE() && STestLastAckResult() == 0x07);
    }

    /* 越界 TARGET 不刷新 */
    {
        STestReset();
        STEST_STALE_MARK();
        STestBuildTarget(f, 0x35);
        /* 把 J1 改为 50m（50000000 µm → 脉冲 1.25e8 > 1e7） */
        uint8_t v[4] = {0x00, 0x40, 0xFA, 0x02};  /* 50000000 LE */
        memcpy(&f[12], v, 4);
        uint16_t len3 = (uint16_t)f[6];
        uint16_t crc3 = Aimotor_CRC16(&f[2], (uint16_t)(6U + len3));
        f[8 + len3] = (uint8_t)(crc3 & 0xFF);
        f[9 + len3] = (uint8_t)(crc3 >> 8);
        Aimotor_HostRxAppend(f, (uint16_t)(10U + len3));
        Aimotor_HostStreamPoll();
        STestReport("越界 TARGET → OUT_OF_RANGE + 不刷新",
                    STEST_STILL_STALE() && STestLastAckResult() == 0x05);
    }

    /* DISABLED 下被拒绝的 TARGET 不刷新 */
    {
        g_control_state = CONTROL_DISABLED;   /* 自检直接置状态（生产函数仍被调用） */
        STestReset();
        STEST_STALE_MARK();
        uint16_t nt = STestBuildTarget(f, 0x36);
        Aimotor_HostRxAppend(f, nt);
        Aimotor_HostStreamPoll();
        STestReport("DISABLED 下 TARGET → STATE_DENIED + 不刷新",
                    STEST_STILL_STALE() && STestLastAckResult() == 0x06);
    }

    /* HELLO 固定不刷新 */
    {
        g_control_state = CONTROL_ENABLED;
        STestReset();
        STEST_STALE_MARK();
        STestBuildFrame(f, 0x01, 0x37, NULL, 0);
        Aimotor_HostRxAppend(f, 10);
        Aimotor_HostStreamPoll();
        STestReport("HELLO 成功但不刷新看门狗（固定）",
                    STEST_STILL_STALE() && STestLastAckResult() == 0x00);
    }

#undef STEST_STALE_MARK
#undef STEST_STILL_STALE

    /* 250ms 超时 → 停止全部轴 + STOPPED；随后 TARGET 拒绝；重新 ENABLE 恢复 */
    {
        g_last_valid_frame_ms = HAL_GetTick() - (HOST_COMM_WATCHDOG_MS + 50U);
        Aimotor_CommWatchdog();
        STestReport("看门狗超时 → 状态 STOPPED + COMM_TIMEOUT 置位",
                    Aimotor_GetControlState() == CONTROL_STOPPED &&
                    (g_control_faults & CONTROL_FAULT_COMM_TIMEOUT) != 0U);
        STestRunSeq();   /* 完成 STOP 序列 */
        STestReset();
        uint16_t nt = STestBuildTarget(f, 0x38);
        Aimotor_HostRxAppend(f, nt);
        Aimotor_HostStreamPoll();
        STestReport("超时后的第一帧 TARGET 拒绝（0x06）", STestLastAckResult() == 0x06);
        STestReset();
        STestBuildFrame(f, 0x02, 0x39, NULL, 0);
        Aimotor_HostRxAppend(f, 10);
        Aimotor_HostStreamPoll();
        STestRunSeq();
        STestReport("重新 ENABLE → ENABLED", Aimotor_GetControlState() == CONTROL_ENABLED);
        STestReset();
        uint16_t nt2 = STestBuildTarget(f, 0x3A);
        Aimotor_HostRxAppend(f, nt2);
        Aimotor_HostStreamPoll();
        STestReport("ENABLE 后 TARGET 恢复允许（0x00）", STestLastAckResult() == 0x00);
    }
    g_test_capture_tx = 0;
}

/* ── 电机安全（P0）：ENABLE 失败→FAULT / 重试耗尽 / STOP 全轴 ─────── */
static void STestMotorSafety(void)
{
    STestOut("\n== 电机安全（生产 CtrlSeqStart / CtrlSeqTick / 全局 STOP）==\r\n");

    /* ENABLE 失败：无回包（reply_wait=1）→ 重试耗尽 → FAULT，不进入 ENABLED */
    STestReset();
    g_test_capture_tx = 1;
    g_test_reply_wait = 1;
    CtrlSeqStart(CTRL_SEQ_ENABLE, 0x40, 0x02, 1);
    for (int i = 0; i < 500 && Aimotor_CtrlSeqActive(); i++) {
        g_ctrl_seq.tx_tick = HAL_GetTick() - 100U;   /* 模拟应答超时 */
        CtrlSeqTick();
    }
    STestReport("ENABLE 全部无应答 → 重试耗尽后 FAULT", Aimotor_GetControlState() == CONTROL_FAULT);
    STestReport("ENABLE 失败 ACK = CTRL_FAILED(0x0A)", STestLastAckResult() == 0x0A);
    STestReport("ENABLE 失败置 ENABLE_FAILED 故障位",
                (g_control_faults & CONTROL_FAULT_ENABLE_FAILED) != 0U);
    STestReport("ENABLE 失败不进入 ENABLED（TARGET 拒绝）",
                Aimotor_GetControlState() != CONTROL_ENABLED);

    /* 上一测试强制等待回包；后续正常 DRY_RUN 安全序列必须恢复立即确认模式。 */
    g_test_reply_wait = 0;

    /* STOP 抢占并覆盖 12 轴：即使序列进行中也立即停止 */
    STestReset();
    CtrlSeqStart(CTRL_SEQ_STOP, 0x41, 0x03, 1);
    STestRunSeq();
    STestReport("全局 STOP 完成 → STOPPED", Aimotor_GetControlState() == CONTROL_STOPPED);
    STestReport("全局 STOP 覆盖 12 轴（12 步全部确认，failed_mask==0）",
                g_ctrl_seq.kind == CTRL_SEQ_NONE &&
                g_ctrl_seq.failed_mask == 0U);
    /* STOP 后 TARGET 必须被拒绝：真实发送 TARGET 帧，验证 ACK=0x06 */
    {
        uint8_t tgt[80];
        uint16_t nt = STestBuildTarget(tgt, 0x42);
        STestReset();
        Aimotor_HostRxAppend(tgt, nt);
        Aimotor_HostStreamPoll();
        STestReport("STOP 后 TARGET 拒绝（ACK 0x06）", STestLastAckResult() == 0x06);
    }

    /* 重试次数有限：重试耗尽后序列结束（不无限重发） */
    STestReset();
    CtrlSeqStart(CTRL_SEQ_STOP, 0, 0, 0);
    int ticks = 0;
    for (int i = 0; i < 5000 && Aimotor_CtrlSeqActive(); i++) {
        g_ctrl_seq.tx_tick = HAL_GetTick() - 100U;
        CtrlSeqTick();
        ticks++;
    }
    STestReport("STOP 无应答 12 轴×4 重试后序列结束（非无限）",
                Aimotor_CtrlSeqActive() == 0 && ticks < 5000);

    /* ENABLE 部分失败回滚：前 4 轴收到成功应答（确认使能），其余 8 轴应答丢失。
       回滚必须对**全部 12 轴**发送安全停止（ServoOff/0x80），
       而不是只处理确认成功轴（应答丢失≠未执行）。 */
    {
        g_test_reply_wait = 1;   /* 序列等待真实回包，逐轴推进 */
        CtrlSeqStart(CTRL_SEQ_ENABLE, 0x43, 0x02, 1);
        /* 阶段 1：ENABLE 前 4 轴注入合法应答，其余 8 轴应答丢失，
           直到 ENABLE 失败并进入回滚序列 */
        int steps_ok = 0;
        for (int i = 0; i < 400 && Aimotor_CtrlSeqActive() &&
                            g_ctrl_seq.kind == CTRL_SEQ_ENABLE; i++) {
            if (g_ctrl_seq.phase == 1 && steps_ok < 4) {
                const SeqStep_t *cs = &g_ctrl_seq.steps[g_ctrl_seq.step];
                if (cs->op == STEP_AI_SERVO_ON) {
                    Aimotor_t *m = &aimotor_motors[cs->side][cs->axis];
                    m->rx_buf[0] = m->slave_id;
                    m->rx_buf[1] = 0x06; m->rx_buf[2] = 0x03; m->rx_buf[3] = 0x03;
                    m->rx_buf[4] = 0x00; m->rx_buf[5] = 0x01;
                    uint16_t crc = Aimotor_CRC16(m->rx_buf, 6);
                    m->rx_buf[6] = (uint8_t)(crc & 0xFF);
                    m->rx_buf[7] = (uint8_t)(crc >> 8);
                    m->rx_len = 8;
                    m->rx_ready = 1;
                    steps_ok++;
                } else if (cs->op == STEP_MW_RUN) {
                    MW_Bus_t *bus = &mw_buses[cs->side];
                    bus->rx_buf[0] = 0x3E; bus->rx_buf[1] = 0x88;
                    bus->rx_buf[2] = mw_motors[cs->side][cs->axis].id;
                    bus->rx_buf[3] = 0x00;
                    bus->rx_buf[4] = MW_Checksum(bus->rx_buf, 4);
                    bus->rx_len = 5;
                    bus->rx_ready = 1;
                    steps_ok++;
                }
            } else {
                g_ctrl_seq.tx_tick = HAL_GetTick() - 100U;   /* 其余轴应答丢失 */
            }
            CtrlSeqTick();
        }
        STestReport("ENABLE 部分失败（4 轴成功应答 + 8 轴丢失）→ 进入回滚",
                    g_ctrl_seq.kind == CTRL_SEQ_ROLLBACK ||
                    Aimotor_GetControlState() == CONTROL_FAULT);
        /* 阶段 2：回滚在 DRY_RUN 下立即确认（每步置 enabled=0），直到完成 */
        g_test_reply_wait = 0;
        STestSeqTraceReset();
        STestRunSeq();
        uint8_t all_off = 1;
        for (uint8_t s = 0; s < AIMOTOR_BUS_COUNT && all_off; s++) {
            for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++)
                if (aimotor_motors[s][m].enabled) { all_off = 0; break; }
            for (uint8_t m = 0; m < MW_MOTORS_PER; m++)
                if (mw_motors[s][m].enabled) { all_off = 0; break; }
        }
        STestReport("回滚覆盖全部 12 轴（含应答丢失轴 enabled==0）", all_off == 1);
        STestReport("回滚真实执行 18 步且覆盖 AI STOP/MW CLOSE/AI ServoOff 全部轴",
                    g_test_seq_send_count == 18U &&
                    g_test_seq_ai_stop_mask == 0x003FU &&
                    g_test_seq_mw_close_mask == 0x003FU &&
                    g_test_seq_ai_off_mask == 0x003FU);
        STestReport("ENABLE 部分失败 ACK = CTRL_FAILED(0x0A)",
                    STestLastAckResult() == 0x0A);
        STestReport("部分失败后状态 FAULT（不进入 ENABLED）",
                    Aimotor_GetControlState() == CONTROL_FAULT);
    }

    /* AI 状态机重试耗尽（真实生产函数 Aimotor_Process）：
       启动一个目标事务，强制应答超时，验证 retry_count 累加至上限后进入
       FAULT（不再无限重发）。owner 逻辑保证 WAIT_* 电机每 tick 被处理。 */
    {
        Aimotor_t *m = &aimotor_motors[0][0];
        m->cmd_pending = 1;
        m->step = MOTOR_STEP_IDLE;
        m->retry_count = 0;
        m->fault = 0;
        int faulted = 0;
        for (int i = 0; i < (AIMOTOR_RETRY_MAX + 8) * 4 && !faulted; i++) {
            m->last_tx_tick = HAL_GetTick() - 100U;   /* 强制应答超时 */
            Aimotor_Process();                          /* 生产函数 */
            if (m->step == MOTOR_STEP_FAULT) faulted = 1;
        }
        STestReport("AI 状态机重试耗尽 → FAULT（Aimotor_Process）",
                    faulted == 1 && m->step == MOTOR_STEP_FAULT);

        /* 故障锁存：即使控制状态仍为 ENABLED，新 TARGET 也必须拒绝，
           不能清除 fault 并自动重启失败事务。 */
        g_control_state = CONTROL_ENABLED;
        STestReset();
        uint8_t tgt[80];
        uint16_t nt = STestBuildTarget(tgt, 0x44);
        Aimotor_HostRxAppend(tgt, nt);
        Aimotor_HostStreamPoll();
        STestReport("AI 轴故障锁存 → 新 TARGET 回 CTRL_FAILED 且不重启",
                    STestLastAckResult() == AIMOTOR_ACK_CTRL_FAILED &&
                    m->fault == 1 && m->step == MOTOR_STEP_FAULT &&
                    m->cmd_pending == 0);
        m->fault = 0;
        m->step = MOTOR_STEP_IDLE;
    }

    /* MW J4 独立轴超时重试（真实生产函数 MW_Process）：
       30ms 应答超时累加到 MW_RETRY_MAX 后结束事务并置 fault。 */
    {
        /* 冻结该总线空闲轮询，防止空闲读干扰本测试 */
        MW_IdlePollSet(0, 1);
        MW_Motor_t *j4 = &mw_motors[0][0];
        j4->cmd_pending = 1;
        j4->step = MW_STEP_IDLE;
        j4->retry_count = 0;
        j4->fault = 0;
        int j4_exhaust = 0;
        for (int i = 0; i < (MW_RETRY_MAX + 8) * 3 && !j4_exhaust; i++) {
            j4->last_tx_tick = HAL_GetTick() - 100U;   /* 强制应答超时 */
            MW_Process();                               /* 生产函数 */
            if (j4->fault && j4->step == MW_STEP_IDLE) j4_exhaust = 1;
        }
        STestReport("MW J4 30ms 超时重试耗尽 → 事务结束（MW_Process）",
                    j4_exhaust == 1);
        MW_IdlePollSet(0, 0);
    }

    /* 文本 STOP 必须能抢占正在等待回包的 ENABLE，而不是排队到序列结束。 */
    {
        STestReset();
        g_test_reply_wait = 1;
        CtrlSeqStart(CTRL_SEQ_ENABLE, 0, 0, 0);
        CtrlSeqTick();   /* 发送第 1 步并进入等待回包 */
        memcpy(g_host_cmd_buf, "ALL STOP", 9);
        g_host_cmd_len = 8;
        g_host_cmd_ready = 1;
        Aimotor_Process();
        STestReport("文本 ALL STOP 抢占等待中的 ENABLE",
                    g_ctrl_seq.kind == CTRL_SEQ_STOP &&
                    Aimotor_GetControlState() == CONTROL_STOPPED);
        g_test_reply_wait = 0;
        STestRunSeq();
    }

    g_test_reply_wait = 0;
    g_test_capture_tx = 0;
}

/* ── 换算与 STATE（生产 MW_ForwardKin / MW_ReadbackUrad / Aimotor_SendState）─ */
static void STestConvertState(void)
{
    STestOut("\n== 换算与 STATE ==\r\n");

    /* J5 含 5/3；右臂取反；J6 耦合 */
    int64_t m4, m5, m6;
    STestReport("左臂 J5=20° → 33333 计数（含 5/3）",
                MW_ForwardKin(0, 0, 20000, 0, &m4, &m5, &m6) == 0 && m5 == 33333);
    STestReport("右臂 J5=20° → -33333", MW_ForwardKin(1, 0, 20000, 0, &m4, &m5, &m6) == 0 && m5 == -33333);
    STestReport("J6 耦合：J5=0,J6=45° → 100000", MW_ForwardKin(0, 0, 0, 45000, &m4, &m5, &m6) == 0 && m6 == 100000);
    STestReport("越界 J5=40000° 拒绝", MW_ForwardKin(0, 0, 40000000, 0, &m4, &m5, &m6) != 0);

    /* 逆换算回环：M5=33333 → J5≈20°（正向 5/3 与逆向 3/5 存在 ±1 deg1000
       截断量化，允许 ±2 deg1000 = ±35 µrad） */
    int64_t j4u, j5u, j6u;
    mw_motors[0][0].actual_angle = 20000;   /* 20°×1000 */
    mw_motors[0][1].actual_angle = 33333;
    mw_motors[0][2].actual_angle = 100000 - 33333;   /* 与目标一致的耦合结果 */
    MW_ReadbackUrad(0, &j4u, &j5u, &j6u);
    STestReport("反馈 J5 逆换算≈20°（±2 deg1000 量化）",
                j5u >= 349031 && j5u <= 349101);
    STestReport("反馈 J6 逆换算≈45°（45°=785398 µrad）",
                j6u >= 785363 && j6u <= 785433);

    /* STATE 12 轴位图与无效位置 0（生产 Aimotor_SendState + 捕获） */
    {
        STestReset();
        g_test_capture_tx = 1;
        for (uint8_t side = 0; side < 2; side++) {
            for (uint8_t j = 0; j < 3; j++) {
                aimotor_motors[side][j].actual_valid = 1;
                aimotor_motors[side][j].actual_position = 1000;
                aimotor_motors[side][j].enabled = 1;
                aimotor_motors[side][j].fault = 0;
                mw_motors[side][j].actual_valid = 1;
                mw_motors[side][j].actual_angle = 1000;
                mw_motors[side][j].enabled = 1;
                mw_motors[side][j].fault = 0;
            }
        }
        /* 关掉 L_J2 与 R_J5 验证无效位置 0 与位图；给 R_J3 置 fault 验证右臂高位 */
        aimotor_motors[0][1].actual_valid = 0;
        mw_motors[1][1].actual_valid = 0;
        aimotor_motors[1][2].fault = 1;   /* R_J3 = bit8（right arm J3） */
        Aimotor_SendState(0x50);
        /* 70 字节：AA55|01|81|seq2|len2=60|payload60|crc2 */
        int ok_total = (g_test_tx_len == 70);
        int ok_len = ok_total && g_test_tx_log[6] == 60 && g_test_tx_log[7] == 0;
        uint16_t valid = (uint16_t)(g_test_tx_log[12] | (g_test_tx_log[13] << 8));
        int ok_bitL1 = (valid & 0x0001) != 0;
        int ok_bitL2 = (valid & 0x0002) == 0;      /* L_J2 无效 */
        int ok_bitR1 = (valid & 0x0040) != 0;      /* R_J1 = bit6 */
        int ok_bitR5 = (valid & 0x0800) == 0;      /* R_J5 = bit10 无效 */
        /* enabled 位图 @[14..15]（新增字段） */
        uint16_t enabled = (uint16_t)(g_test_tx_log[14] | (g_test_tx_log[15] << 8));
        int ok_enL1 = (enabled & 0x0001) != 0;
        int ok_enR3 = (enabled & 0x0100) != 0;     /* R_J3 = bit8 */
        /* axis_status uint32 @[16..19]：R_J3 轴序 = side*6+joint = 1*6+2 = 8，
           fault 位 = 2 << (2*8) = 2<<16（bit17） */
        uint32_t axis_status = (uint32_t)g_test_tx_log[16] |
                               ((uint32_t)g_test_tx_log[17] << 8) |
                               ((uint32_t)g_test_tx_log[18] << 16) |
                               ((uint32_t)g_test_tx_log[19] << 24);
        int ok_stR3fault = (axis_status & (2UL << 16)) != 0;   /* R_J3 fault */
        /* 位置区从 payload 偏移 12（f[20]）起（seq2+state+fault+valid2+enabled2+status4） */
        int32_t posL1 = (int32_t)(g_test_tx_log[20] | (g_test_tx_log[21] << 8) |
                                  (g_test_tx_log[22] << 16) | ((uint32_t)g_test_tx_log[23] << 24));
        int32_t posL2 = (int32_t)(g_test_tx_log[24] | (g_test_tx_log[25] << 8) |
                                  (g_test_tx_log[26] << 16) | ((uint32_t)g_test_tx_log[27] << 24));
        /* L_J1 = 1000 脉冲 → µm = 1000*4/10 = 400 */
        int ok_posL1 = posL1 == 400;
        int ok_posL2 = posL2 == 0;
        /* CRC 校验（覆盖 6+60=66 字节） */
        uint16_t calc = Aimotor_CRC16(&g_test_tx_log[2], 6U + 60U);
        int ok_crc = (g_test_tx_log[68] == (calc & 0xFF)) && (g_test_tx_log[69] == (calc >> 8));
        STestReport("STATE 总长 70 + LEN=60 + SEQ 正确",
                    ok_total && ok_len && g_test_tx_log[4] == 0x50 && g_test_tx_log[5] == 0);
        STestReport("STATE 位图：L1 置位 / L2 清除 / R1(bit6) 置位 / R5(bit10) 清除",
                    ok_bitL1 && ok_bitL2 && ok_bitR1 && ok_bitR5);
        STestReport("STATE enabled 位图：L1/R3 置位（新增字段）", ok_enL1 && ok_enR3);
        STestReport("STATE axis_status uint32：右臂 R_J3 fault 位(bit17) 不丢失",
                    ok_stR3fault);
        STestReport("STATE 无效轴位置为 0，有效轴为换算值",
                    ok_posL1 && ok_posL2);
        STestReport("STATE CRC 覆盖 66 字节且通过", ok_crc);
        g_test_capture_tx = 0;
    }
}

/* ── DRY_RUN：真实发送计数恒 0，mock 有记录 ─────────────────────── */
static void STestDryRun(void)
{
    STestOut("\n== DRY_RUN 全局屏蔽（生产发送函数）==\r\n");
    /* 通过 ENABLE 序列驱动一次 AI+MW 发送路径 */
    STestReset();
    g_dry_run_log_index = 0;
    for (uint8_t i = 0; i < AIMOTOR_DRY_RUN_LOG_MAX; i++) {
        g_dry_run_log[i].len = 0;
        g_dry_run_log[i].is_mw = 0;
    }
    g_test_capture_tx = 1;
    STestBuildFrame(g_test_tx_log, 0x02, 0x60, NULL, 0);
    /* 直接构造 ENABLE 帧 */
    uint8_t f[80];
    STestBuildFrame(f, 0x02, 0x60, NULL, 0);
    Aimotor_HostRxAppend(f, 10);
    Aimotor_HostStreamPoll();
    STestRunSeq();
    STestReport("DRY_RUN：AI 真实发送计数 == 0", g_dry_run_ai_tx_count == 0);
    STestReport("DRY_RUN：MW 真实发送计数 == 0", g_dry_run_mw_tx_count == 0);
    STestReport("DRY_RUN：mock 记录了 ENABLE 命令", g_dry_run_log_index != 0);
    STestReport("DRY_RUN：参数/状态检查照常（ENABLE→ENABLED）",
                Aimotor_GetControlState() == CONTROL_ENABLED);
    g_test_capture_tx = 0;
}

void Aimotor_SelfTest(void)
{
    /* 先完成 Aimotor_Init 启动的上电安全停止序列（DRY_RUN 下立即完成），
       避免与后续测试的 ENABLE 序列冲突。 */
    STestRunSeq();

    STestOut("\n===== STM32 固件自检 (AIMOTOR_SELF_TEST) =====");
    STestOut("\r\n宏：DRY_RUN=1（强制）, TEXT_MOTION_ENABLED="
             AIMOTOR_XSTR(AIMOTOR_TEXT_MOTION_ENABLED) ", SELF_TEST=1");
    STestOut("\r\n");

    STestParser();
    STestWatchdog();
    STestMotorSafety();
    STestConvertState();
    STestDryRun();

    char buf[64];
    int n = snprintf(buf, sizeof(buf),
                     "\r\n===== 自检结果：%d passed, %d failed =====\r\n",
                     g_test_pass, g_test_fail);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
}

#endif /* AIMOTOR_SELF_TEST */

/* USER CODE END 0 */
