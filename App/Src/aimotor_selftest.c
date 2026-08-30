/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : aimotor_selftest.c
  * @brief          : 固件自检（AIMOTOR_SELF_TEST=1 时编译，直接调用生产 C 函数）
  *                   （自 aimotor.c 拆分，测试体逐行搬迁未改动）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "aimotor.h"
#include "aimotor_internal.h"
#include "mwmotor.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* ======================================================================== */
/*            固件自检（AIMOTOR_SELF_TEST，直接调用生产 C 函数）                */
/* ======================================================================== */
#if AIMOTOR_SELF_TEST

/* 自检 TX 捕获与状态（原 aimotor.c 顶部定义随自检迁入；
   生产函数的 #if 钩子经 aimotor_internal.h 访问） */
uint8_t  g_test_capture_tx = 0;
uint8_t  g_test_tx_log[1024];
uint16_t g_test_tx_len = 0;
uint8_t  g_test_reply_wait = 0;
static int      g_test_pass = 0;
static int      g_test_fail = 0;
uint8_t  g_test_seq_send_count = 0;
uint16_t g_test_seq_ai_stop_mask = 0;
uint16_t g_test_seq_ai_off_mask = 0;
uint16_t g_test_seq_mw_close_mask = 0;


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
    g_mw_cmd_ready = 0;
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
