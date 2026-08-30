/**
  ******************************************************************************
  * @file           : aimotor_internal.h
  * @brief          : aimotor 家族内部共享符号（跨 .c 文件，不对 main.c/外部暴露）
  *
  * 拆分布局（App/Src）：
  *   aimotor.c        — 电机数组/初始化/DMA 回调路由/单电机状态机（Aimotor_Process）
  *   aimotor_modbus.c — Modbus RTU 编解码与 RS485 发送
  *   aimotor_ctrl.c   — 异步控制序列（ENABLE/STOP/DISABLE/上电安全停止/回滚）
  *   host_protocol.c  — 上位机协议层（流解析/二进制帧/看门狗/文本命令/控制状态）
  *   aimotor_selftest.c — 固件自检（AIMOTOR_SELF_TEST=1 时编译）
  *   comm_router.c    — HAL UART 回调分发
  * 本头文件只被上述 .c 互相引用，main.c 与 mwmotor.c 仅经 aimotor.h /
  * host_protocol.h 的公共 API 交互。
  ******************************************************************************
  */

#ifndef __AIMOTOR_INTERNAL_H__
#define __AIMOTOR_INTERNAL_H__

#include "main.h"
#include "aimotor.h"

/* 编译期配置打印辅助（aimotor.c pragma message / 自检输出共用） */
#define AIMOTOR_XSTR_(x) #x
#define AIMOTOR_XSTR(x)  AIMOTOR_XSTR_(x)

/* ── 控制状态机（定义在 host_protocol.c，协议层与控制序列两层共用） ── */
extern ControlState_t g_control_state;
extern uint8_t        g_control_faults;
extern uint32_t       g_last_valid_frame_ms;   /* 上次有效命令时间戳 */

/* ── 上位机命令槽（定义在 host_protocol.c，原 main.c 定义随协议层迁入） ──
   g_host_cmd_*：B/ALL 文本命令，由 Aimotor_Process 消费；
   g_mw_cmd_* ：L/R 文本命令（原名 g_host_rx_*，消灭旧别名后更名），
                由 MW_Process 消费；host_protocol.c 的流解析负责填充。 */
extern char              g_host_cmd_buf[64];
extern volatile uint16_t g_host_cmd_len;
extern volatile uint8_t  g_host_cmd_ready;
extern uint8_t           g_mw_cmd_buf[64];
extern volatile uint16_t g_mw_cmd_len;
extern volatile uint8_t  g_mw_cmd_ready;

/* USART1 DMA 接收暂存（comm_router.c 回调写入，host_protocol.c 重启 DMA） */
extern uint8_t g_host_dma_buf[64];

/* ── 上位机流解析缓冲状态（定义在 host_protocol.c；自检直接构造解析场景） ── */
extern volatile uint16_t g_host_ring_head;   /* 下一个写入位置 */
extern volatile uint16_t g_host_ring_tail;   /* 下一个读取位置 */
extern uint16_t          g_host_text_len;

/* ── 异步控制序列（定义在 aimotor_ctrl.c） ────────────────────── */
/* 每个序列最多步骤数（DISABLE = 6 停止 + 6 关闭 + 6 ServoOff = 18） */
#define CTRL_SEQ_MAX_STEPS 18

typedef enum {
    STEP_AI_SERVO_ON,   /* 总线使能 0x0303=1 */
    STEP_AI_SERVO_OFF,  /* 总线断开 0x0303=0 */
    STEP_AI_STOP,       /* 停止 0x0305=0 */
    STEP_MW_RUN,        /* 0x88 运行 */
    STEP_MW_CLOSE       /* 0x80 关闭 */
} SeqOp_t;

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

extern CtrlSeq_t g_ctrl_seq;

void CtrlSeqStart(CtrlSeqKind_t kind, uint16_t seq, uint8_t cmd,
                  uint8_t has_ack);
void CtrlSeqTick(void);
/* 清空全部待执行目标与重试任务（STOP/DISABLE/看门狗/新事务开始时调用） */
void ClearAllPending(void);

/* ── 协议层内部（定义在 host_protocol.c） ── */
void BinarySendAck(uint16_t seq, uint8_t cmd, uint8_t result);
void HostCmd_Execute(const Aimotor_Cmd_t *cmd);

/* Modbus 编解码（定义在 aimotor_modbus.c） */
int32_t Aimotor_DecodeInt32CDAB(const uint8_t data[4]);

/* ── 自检钩子（定义在 aimotor_selftest.c；生产函数的 #if 块访问） ── */
#if AIMOTOR_SELF_TEST
extern uint8_t  g_test_capture_tx;
extern uint8_t  g_test_tx_log[1024];
extern uint16_t g_test_tx_len;
extern uint8_t  g_test_reply_wait;   /* 1: DRY_RUN 下序列也等待回包（测重试/超时） */
extern uint8_t  g_test_seq_send_count;
extern uint16_t g_test_seq_ai_stop_mask;
extern uint16_t g_test_seq_ai_off_mask;
extern uint16_t g_test_seq_mw_close_mask;
#endif

#endif /* __AIMOTOR_INTERNAL_H__ */
