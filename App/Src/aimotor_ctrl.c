/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : aimotor_ctrl.c
  * @brief          : 异步控制序列状态机（ENABLE/STOP/DISABLE/上电安全停止/回滚）
  *                   （自 aimotor.c 拆分，函数体逐行搬迁未改动）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "aimotor.h"
#include "aimotor_internal.h"
#include "mwmotor.h"

/* 控制序列状态（类型定义见 aimotor_internal.h） */
CtrlSeq_t g_ctrl_seq;

/* 前向声明：CtrlSeqFinish 的 ENABLE 失败路径先于定义处调用回滚构建 */
static void CtrlSeqBuildRollback(void);

/* 清空全部待执行目标与重试任务（STOP/DISABLE/看门狗/新事务开始时调用） */
void ClearAllPending(void)
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
void CtrlSeqStart(CtrlSeqKind_t kind, uint16_t seq, uint8_t cmd,
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
void CtrlSeqTick(void)
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
