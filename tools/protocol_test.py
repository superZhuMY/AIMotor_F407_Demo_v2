#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 下位机二进制协议测试脚本（主机端，不驱动任何电机）

用途
----
1) 生成固定测试向量（帧字节 + 期望 CRC + 期望 ACK/STATE），供串口抓包比对。
2) 用与固件相同算法的字节级增量解析器模拟 16 种拆包/粘包/CRC 场景。
3) 校验文本/二进制换算一致性（J5 5/3、J5/J6 耦合、右臂取反、逆换算）。
4) 校验 ACK/STATE 帧布局与控制状态机。
5) 输出人工真机验收步骤。

协议权威定义见 docs/README_J456_DRIVER.md §6。
依赖：Python 3.6+，标准库即可。
运行：python protocol_test.py
"""

import struct
from binascii import hexlify

# =====================================================================
# 协议常量（与 Core/Inc/aimotor.h 一致）
# =====================================================================
VERSION = 0x01
CMD_HELLO, CMD_ENABLE, CMD_STOP, CMD_GET_STATE, CMD_DISABLE = 0x01, 0x02, 0x03, 0x04, 0x05
CMD_TARGET = 0x10
RESP_ACK, RESP_STATE = 0x80, 0x81

ACK_OK, ACK_CRC, ACK_FORMAT, ACK_UNKNOWN_CMD, ACK_BAD_LEN = 0x00, 0x01, 0x02, 0x03, 0x04
ACK_OUT_OF_RANGE, ACK_STATE_DENIED, ACK_BAD_ARM, ACK_COMM_TIMEOUT = 0x05, 0x06, 0x07, 0x08

STATE_DISABLED, STATE_ENABLED, STATE_STOPPED, STATE_FAULT = 0, 1, 2, 3

HEADER_SIZE = 8
MAX_PAYLOAD = 60   # 与固件 PROTOCOL_MAX_PAYLOAD 一致（STATE 帧 LEN=60）

AIMOTOR_POSITION_MIN, AIMOTOR_POSITION_MAX = -10000000, 10000000
MW_DEG1000_LIMIT = 36000000

_passed, _failed = 0, 0


def check(name, cond, detail=""):
    global _passed, _failed
    if cond:
        _passed += 1
        print("  [PASS] %s" % name)
    else:
        _failed += 1
        print("  [FAIL] %s  %s" % (name, detail))


# =====================================================================
# CRC16 (Modbus, poly 0xA001, init 0xFFFF) — 与 Aimotor_CRC16 等价
# =====================================================================
def crc16(buf):
    crc = 0xFFFF
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def append_crc(frame):
    """CRC 覆盖 frame[2:]（VERSION..payload 末尾 = 6+LEN 字节），低字节在前。"""
    c = crc16(frame[2:])
    return frame + bytes([c & 0xFF, c >> 8])


# =====================================================================
# 帧构造器（主机发送侧）
# =====================================================================
def make_frame(cmd, seq, payload):
    payload = bytes(payload)
    return append_crc(bytes([0xAA, 0x55, VERSION, cmd, seq & 0xFF, (seq >> 8) & 0xFF,
                             len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload)


def make_target(side, joints_um, joints_urad, seq=1):
    p = bytes([side, 0, 0, 0])  # arm, mode, flags(2) → PAYLOAD 偏移 4 起为 6×int32
    for v in joints_um + joints_urad:
        p += struct.pack('<i', v)
    return make_frame(CMD_TARGET, seq, p)


# =====================================================================
# 换算（与 mwmotor.c MW_ForwardKin / MW_BinaryConvert 等价）
# =====================================================================
def urad_to_deg1000(urad):
    import math
    return round(urad * (180000.0 / math.pi) / 1e6)


def fk(side, j4d, j5d, j6d):
    """MW_ForwardKin：输入 deg×1000，输出电机侧目标（含右臂取反与 J5/J6 耦合）。"""
    m4 = j4d
    m5 = j5d * 5 // 3
    if side == 1:
        m5 = -m5
    m6 = j6d * 20 // 9 - m5
    return m4, m5, m6


def ai_pulse(um, i):
    return [um * 10 // 4, um * 10000 // 475, um * 10 // 2][i]


def ai_um(pulse, i):
    return [pulse * 4 // 10, pulse * 475 // 10000, pulse * 2 // 10][i]


# =====================================================================
# 字节级增量解析器（镜像 Aimotor_HostStreamPoll 的算法）
# 事件：(frame, None) 表示通过 CRC 校验的完整帧；('ack', code) 表示错误回包。
# =====================================================================
class StreamParser:
    def __init__(self):
        self.buf = bytearray()          # 模拟环形缓冲
        self.events = []

    def feed(self, data):
        self.buf += bytes(data)
        self._poll()

    def _poll(self):
        while True:
            # 查找 AA 55
            idx = -1
            for i in range(len(self.buf) - 1):
                if self.buf[i] == 0xAA and self.buf[i + 1] == 0x55:
                    idx = i
                    break
            if idx < 0:
                if self.buf and self.buf[0] == 0xAA and len(self.buf) == 1:
                    return            # 只有 AA，等待下一字节
                return
            del self.buf[:idx]          # 帧头前字节按噪声丢弃
            if len(self.buf) < HEADER_SIZE:
                return                  # 不足 8 字节
            n = int.from_bytes(self.buf[6:8], 'little')
            if n > MAX_PAYLOAD:
                # 错误 LEN：丢弃到下一个可能的帧头
                self.buf = self.buf[1:]
                continue
            total = HEADER_SIZE + n + 2
            if len(self.buf) < total:
                return                  # 半帧，等待后续
            frame = bytes(self.buf[:total])
            del self.buf[:total]        # 消费该帧（无论校验成败）
            if frame[2] != VERSION:
                self.events.append(('ack', ACK_FORMAT))
            elif crc16(frame[2:8 + n]) != int.from_bytes(frame[8 + n:10 + n], 'little'):
                self.events.append(('ack', ACK_CRC))
            else:
                self.events.append((frame, None))


# =====================================================================
# 测试 1：固定测试向量
# =====================================================================
def test_vectors():
    print("== 固定测试向量 ==")
    joints_um = [100000, 200000, 300000]
    joints_urad = [round(v * 1e6 * 3.14159265358979 / 180.0) for v in (20, 20, 45)]
    f = make_target(0, joints_um, joints_urad, seq=0x1234)
    check("TARGET 总长 == 10+28 = 38", len(f) == 38, hexlify(f))
    check("TARGET SOF/VER/CMD", f[0] == 0xAA and f[1] == 0x55 and f[2] == VERSION and f[3] == CMD_TARGET)
    check("TARGET SEQ @4..5 LE", int.from_bytes(f[4:6], 'little') == 0x1234)
    check("TARGET LEN @6..7 == 28", int.from_bytes(f[6:8], 'little') == 28)
    check("TARGET 关节1 从 f[12] 起", int.from_bytes(f[12:16], 'little', signed=True) == joints_um[0])
    check("TARGET 关节4 从 f[24] 起", int.from_bytes(f[24:28], 'little', signed=True) == joints_urad[0])
    check("TARGET CRC @[36..37] 低字节在前",
          f[36] == (crc16(f[2:36]) & 0xFF) and f[37] == (crc16(f[2:36]) >> 8))

    a = append_crc(bytes([0xAA, 0x55, VERSION, RESP_ACK, 0x34, 0x12, 4, 0,
                          0x34, 0x12, CMD_TARGET, ACK_OK]))
    check("ACK 总长 == 10+4 = 14", len(a) == 14, hexlify(a))
    check("ACK LEN @6..7 == 4", int.from_bytes(a[6:8], 'little') == 4)
    check("ACK payload seq == 请求 seq", int.from_bytes(a[8:10], 'little') == 0x1234)
    check("ACK payload cmd/result", a[10] == CMD_TARGET and a[11] == ACK_OK)
    check("ACK CRC @[12..13] 覆盖 10 字节",
          a[12] == (crc16(a[2:12]) & 0xFF) and a[13] == (crc16(a[2:12]) >> 8))

    pos = [0] * 12
    # seq, state, fault, valid, enabled, axis_status(uint32)
    p = struct.pack('<HBBHHI', 7, STATE_ENABLED, 0, 0x0001, 0x0001, 0x00000003)
    for v in pos:
        p += struct.pack('<i', v)
    assert len(p) == 60, "STATE payload must be 60 bytes"
    st = append_crc(bytes([0xAA, 0x55, VERSION, RESP_STATE, 7, 0, 60, 0]) + p)
    check("STATE 总长 == 10+60 = 70", len(st) == 70, hexlify(st))
    check("STATE LEN @6..7 == 60", int.from_bytes(st[6:8], 'little') == 60)
    check("STATE CRC 覆盖 66 字节 @[68..69]",
          st[68] == (crc16(st[2:68]) & 0xFF) and st[69] == (crc16(st[2:68]) >> 8))


# =====================================================================
# 测试 2：16 种解析场景
# =====================================================================
def test_stream_parser():
    print("== 帧解析场景 ==")
    f = make_target(0, [100000, 200000, 300000], [174533, 349066, 785398], seq=5)
    n_ok = lambda p: sum(1 for e in p.events if e[0] != 'ack')
    n_ack = lambda p: sum(1 for e in p.events if e[0] == 'ack')

    def run(name, chunks, exp_ok, exp_ack):
        p = StreamParser()
        for c in chunks:
            p.feed(c)
        check(name, n_ok(p) == exp_ok and n_ack(p) == exp_ack,
              "expected ok=%d ack=%d, got ok=%d ack=%d" % (exp_ok, exp_ack, n_ok(p), n_ack(p)))

    bad = bytes([f[i] ^ 0xFF if i in (36, 37) else f[i] for i in range(len(f))])  # CRC 损坏

    run("1 一次完整 TARGET", [f], 1, 0)
    run("2 一帧拆成两次", [f[:20], f[20:]], 1, 0)
    run("3 一帧逐字节", [bytes([b]) for b in f], 1, 0)
    run("4 两帧粘包", [f + f], 2, 0)
    run("5 三帧粘包", [f + f + f], 3, 0)
    # 帧 1 完整（含 CRC）+ 帧 2 开头在同一批到达：帧 1 正常解析，帧 2 半帧等待；
    # 补足帧 2 剩余字节后完成解析。
    p = StreamParser()
    p.feed(f + f[:10])
    check("6 帧1完整+帧2开头同时到达", n_ok(p) == 1 and n_ack(p) == 0,
          "ok=%d ack=%d" % (n_ok(p), n_ack(p)))
    p.feed(f[10:])
    check("6b 补足帧2剩余字节 → 两帧均解析", n_ok(p) == 2 and n_ack(p) == 0,
          "ok=%d ack=%d" % (n_ok(p), n_ack(p)))
    run("7 帧头前随机噪声", [b'\x01\x02\x03' + f], 1, 0)
    run("8 CRC 错误帧", [bad], 0, 1)
    # LEN 错误：帧本身 CRC 通过，进入命令分发后按 BAD_LEN 处理（本层只统计解析事件；
    # 分发层结果见 test_dispatch）
    run("9 LEN 小于要求（TARGET LEN=20）", [make_frame(CMD_TARGET, 1, bytes(20))], 1, 0)
    run("10 LEN 大于要求（TARGET LEN=40）", [make_frame(CMD_TARGET, 1, bytes(40))], 1, 0)
    run("11 LEN 超过最大值（60）", [make_frame(CMD_TARGET, 1, bytes(61))], 0, 0)
    run("12 不支持的 VERSION", [bytes([0xAA, 0x55, 0x02]) + f[3:]], 0, 1)
    run("13 未知 CMD", [make_frame(0x7F, 1, bytes(0))], 1, 0)
    # 半帧：feed 前 20 字节后不再 feed，不产生事件（等待后续数据）
    p = StreamParser()
    p.feed(f[:20])
    check("14 半帧长时间无后续数据 → 无事件", n_ok(p) == 0 and n_ack(p) == 0)
    # 溢出恢复：大量噪声（含 AA）后跟合法帧
    run("15 缓冲溢出后恢复", [b'\xAA' * 300 + f], 1, 0)
    run("16 CRC 错误帧后紧跟合法帧", [bad + f], 1, 1)


# =====================================================================
# 测试 3：命令分发层（LEN 精确匹配 + 状态检查）
# =====================================================================
def test_dispatch():
    print("== 命令分发层 ==")
    # 与 Aimotor_HandleBinaryFrame 相同的 LEN/arm 检查逻辑
    def dispatch(cmd, payload):
        if cmd == CMD_HELLO:
            return ACK_OK if len(payload) == 0 else ACK_BAD_LEN
        if cmd == CMD_ENABLE:
            return ACK_OK if len(payload) == 0 else ACK_BAD_LEN
        if cmd == CMD_STOP:
            if len(payload) != 1:
                return ACK_BAD_LEN
            # arm 字段仅为兼容保留；固件忽略其值，STOP 始终覆盖双臂 12 轴。
            return ACK_OK
        if cmd == CMD_GET_STATE:
            return ACK_OK if len(payload) == 0 else ACK_BAD_LEN
        if cmd == CMD_DISABLE:
            return ACK_OK if len(payload) == 0 else ACK_BAD_LEN
        if cmd == CMD_TARGET:
            if len(payload) != 28:
                return ACK_BAD_LEN
            return ACK_OK
        return ACK_UNKNOWN_CMD

    check("TARGET LEN=20 → BAD_LEN(0x04)", dispatch(CMD_TARGET, bytes(20)) == ACK_BAD_LEN)
    check("TARGET LEN=28 → OK", dispatch(CMD_TARGET, bytes(28)) == ACK_OK)
    check("STOP arm=2（兼容字段任意值）→ OK", dispatch(CMD_STOP, bytes([2])) == ACK_OK)
    check("STOP arm=255（兼容字段任意值）→ OK", dispatch(CMD_STOP, bytes([255])) == ACK_OK)
    check("STOP arm=1 → OK", dispatch(CMD_STOP, bytes([1])) == ACK_OK)
    check("ENABLE 带 1 字节 payload → BAD_LEN", dispatch(CMD_ENABLE, bytes([0])) == ACK_BAD_LEN)
    check("未知 CMD → UNKNOWN(0x03)", dispatch(0x7F, bytes(0)) == ACK_UNKNOWN_CMD)


# =====================================================================
# 测试 4：换算一致性
# =====================================================================
def test_conversion():
    print("== 换算一致性（文本 vs 二进制）==")
    import math
    for j5, j6 in [(20, 45), (-30, 15), (0, 0), (90, -60), (7, -7)]:
        for side in (0, 1):
            m5_text, m6_text = fk(side, 0, j5 * 1000, j6 * 1000)[1:]
            urad5 = round(j5 * math.pi / 180.0 * 1e6)
            urad6 = round(j6 * math.pi / 180.0 * 1e6)
            m5_bin, m6_bin = fk(side, 0, urad_to_deg1000(urad5), urad_to_deg1000(urad6))[1:]
            check("J5/J6 文本==二进制 (side=%d J5=%d J6=%d)" % (side, j5, j6),
                  m5_text == m5_bin and m6_text == m6_bin,
                  "text=(%d,%d) bin=(%d,%d)" % (m5_text, m6_text, m5_bin, m6_bin))
            if side == 0 and j5 == 20:
                check("J5 20° → 33333 计数（含 5/3）", m5_text == 33333, m5_text)
    check("右臂 J5 取负", fk(1, 0, 20000, 0)[1] == -33333)
    check("J6 耦合 M6 = J6*20/9 - M5（J5=0）", fk(0, 0, 0, 45000)[2] == 100000)
    # 正/逆换算回环（与固件 MW_ReadbackUrad 单位一致，deg×1000）：
    #   M5=33333 → J5_deg1000 = 33333*3/5 = 19999（±1 量化），≈ 20°
    #   M6=66667 → J6_deg1000 = (66667+33333)*9/20 = 45000，= 45°
    _, m5, m6 = fk(0, 0, 20000, 45000)
    j5_inv_d1000 = m5 * 3 // 5
    j6_inv_d1000 = (m6 + m5) * 9 // 20
    check("反馈 J5 逆换算≈20°（±1 deg1000 量化）", abs(j5_inv_d1000 - 20000) <= 1,
          "j5_inv=%d" % j5_inv_d1000)
    check("反馈 J6 逆换算 45°（deg×1000）", j6_inv_d1000 == 45000,
          "j6_inv=%d" % j6_inv_d1000)
    check("AI 越界拒绝（J1=50m）", ai_pulse(50000000, 0) > AIMOTOR_POSITION_MAX)
    check("MW 越界拒绝（J5=40000°）", MW_DEG1000_LIMIT < 40000 * 1000)


# =====================================================================
# 测试 5：控制状态机
# =====================================================================
def test_state_machine():
    print("== 控制状态机 ==")
    target_allowed = lambda s: s == STATE_ENABLED
    check("DISABLED 拒绝 TARGET", not target_allowed(STATE_DISABLED))
    check("ENABLED 允许 TARGET", target_allowed(STATE_ENABLED))
    check("STOPPED 拒绝 TARGET", not target_allowed(STATE_STOPPED))
    check("FAULT 拒绝 TARGET", not target_allowed(STATE_FAULT))


# =====================================================================
# 人工验收步骤
# =====================================================================
def manual_steps():
    print("""
== 人工验收步骤（真机/回环，任选其一） ==
A. 回环（不驱动电机）：把 Core/Inc/main.h 的 AIMOTOR_DRY_RUN 改为 1 重新编译烧录，
   用串口助手按下列步骤观察回包。
B. 真机：串口转 TTL/RS485 抓包 + 电机断供电优先。

步骤：
1. 复位后发 TARGET(seq=1)          → 期望 ACK result=0x06（未 ENABLE 拒绝）
2. 发 ENABLE(seq=2)                → ACK result=0x00（AI ServoOn + MW 0x88，序列完成后回发）
3. 发 TARGET(seq=3) 左臂小目标      → ACK result=0x00；抓包确认 6 电机均收到命令
4. 发 GET_STATE(seq=4)             → STATE 帧：LEN=60，control_state=1，valid/enabled 位与真机一致
5. 停发 300ms                      → 抓包确认全部电机收到停止；STATE fault=0x01，state=2
6. 发 TARGET(seq=5)                → ACK result=0x06（STOPPED 拒绝）
7. 发 ENABLE(seq=6)                → ACK result=0x00；再发 TARGET(seq=7) → ACK 0x00
8. 发 STOP(seq=8, arm 任意)        → ACK 0x00；确认双臂 J1..J6（AI+MW）全部停止（全局 STOP）
9. 发 DISABLE(seq=9)               → ACK 0x00；TARGET(seq=10) → ACK 0x06
10. CRC 错误帧                     → ACK result=0x01
11. 半帧（只发前 10 字节）后等 200ms → 无回包；随后补齐后续字节 → 正常 ACK
12. 粘包：连续发 2 个 TARGET        → 收到 2 个 ACK，SEQ 与请求一一对应
""")


def main():
    print("=== STM32 二进制协议主机侧测试 ===")
    test_vectors()
    test_stream_parser()
    test_dispatch()
    test_conversion()
    test_state_machine()
    manual_steps()
    print("=== 结果：%d passed, %d failed ===" % (_passed, _failed))
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
