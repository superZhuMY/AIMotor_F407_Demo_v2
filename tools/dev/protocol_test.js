#!/usr/bin/env node
// -*- coding: utf-8 -*-
/**
 * protocol_test.py 的 Node.js 逐行等价移植（本机无 Python，见同目录 README.md）。
 *
 * 与 tools/protocol_test.py 保持 1:1 断言、1:1 计数值；Python 语义差异点已显式对齐：
 *   - `//`（向下取整）→ pdiv()
 *   - round()（半值向偶数）→ pround()
 *   - math.pi → Math.PI（同一 double 常量，运算顺序保持一致以逐位对齐）
 * 用途与用法见 tools/protocol_test.py 头部注释。
 * 退出码：0 = 全部通过，1 = 有失败。
 */
'use strict';

// =====================================================================
// 协议常量（与 Core/Inc/aimotor.h 一致）
// =====================================================================
const VERSION = 0x01;
const CMD_HELLO = 0x01, CMD_ENABLE = 0x02, CMD_STOP = 0x03, CMD_GET_STATE = 0x04, CMD_DISABLE = 0x05;
const CMD_TARGET = 0x10;
const RESP_ACK = 0x80, RESP_STATE = 0x81;

const ACK_OK = 0x00, ACK_CRC = 0x01, ACK_FORMAT = 0x02, ACK_UNKNOWN_CMD = 0x03, ACK_BAD_LEN = 0x04;
const ACK_OUT_OF_RANGE = 0x05, ACK_STATE_DENIED = 0x06, ACK_BAD_ARM = 0x07, ACK_COMM_TIMEOUT = 0x08;

const STATE_DISABLED = 0, STATE_ENABLED = 1, STATE_STOPPED = 2, STATE_FAULT = 3;

const HEADER_SIZE = 8;
const MAX_PAYLOAD = 60;   // 与固件 PROTOCOL_MAX_PAYLOAD 一致（STATE 帧 LEN=60）

const AIMOTOR_POSITION_MIN = -10000000, AIMOTOR_POSITION_MAX = 10000000;
const MW_DEG1000_LIMIT = 36000000;

let _passed = 0, _failed = 0;

function check(name, cond, detail = '') {
  if (cond) {
    _passed++;
    console.log('  [PASS] ' + name);
  } else {
    _failed++;
    console.log('  [FAIL] ' + name + '  ' + detail);
  }
}

// Python 语义辅助
const pdiv = (a, b) => Math.floor(a / b);            // 整数 `//`
function pround(x) {                                  // round() 半值向偶数
  const f = Math.floor(x);
  const d = x - f;
  if (d > 0.5) return f + 1;
  if (d < 0.5) return f;
  return (f % 2 === 0) ? f : f + 1;
}
const hex = (arr) => Buffer.from(arr).toString('hex');

// =====================================================================
// CRC16 (Modbus, poly 0xA001, init 0xFFFF) — 与 Aimotor_CRC16 等价
// =====================================================================
function crc16(buf) {
  let crc = 0xFFFF;
  for (const b of buf) {
    crc ^= b;
    for (let i = 0; i < 8; i++) {
      crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
  }
  return crc & 0xFFFF;
}

function append_crc(frame) {
  const c = crc16(frame.slice(2));
  return cat(frame, [c & 0xFF, c >> 8]);
}

// =====================================================================
// 帧构造器（主机发送侧）
// =====================================================================
function cat(...arrs) {
  const out = [];
  for (const a of arrs) for (const b of a) out.push(b);
  return out;
}

function make_frame(cmd, seq, payload) {
  payload = Array.from(payload);
  return append_crc(cat([
    0xAA, 0x55, VERSION, cmd, seq & 0xFF, (seq >> 8) & 0xFF,
    payload.length & 0xFF, (payload.length >> 8) & 0xFF,
  ], payload));
}

function i32le(v) {
  const b = Buffer.alloc(4);
  b.writeInt32LE(v, 0);
  return Array.from(b);
}

function make_target(side, joints_um, joints_urad, seq = 1) {
  let p = [side, 0, 0, 0];  // arm, mode, flags(2) → PAYLOAD 偏移 4 起为 6×int32
  for (const v of joints_um.concat(joints_urad)) p = p.concat(i32le(v));
  return make_frame(CMD_TARGET, seq, p);
}

// =====================================================================
// 换算（与 mwmotor.c MW_ForwardKin / MW_BinaryConvert 等价）
// =====================================================================
function urad_to_deg1000(urad) {
  return pround(urad * (180000.0 / Math.PI) / 1e6);
}

function fk(side, j4d, j5d, j6d) {
  const m4 = j4d;
  let m5 = pdiv(j5d * 5, 3);
  if (side === 1) m5 = -m5;
  const m6 = pdiv(j6d * 20, 9) - m5;
  return [m4, m5, m6];
}

function ai_pulse(um, i) {
  return [pdiv(um * 10, 4), pdiv(um * 10000, 475), pdiv(um * 10, 2)][i];
}

function ai_um(pulse, i) {
  return [pdiv(pulse * 4, 10), pdiv(pulse * 475, 10000), pdiv(pulse * 2, 10)][i];
}

// =====================================================================
// 字节级增量解析器（镜像 Aimotor_HostStreamPoll 的算法）
// =====================================================================
class StreamParser {
  constructor() {
    this.buf = [];       // 模拟环形缓冲
    this.events = [];
  }

  feed(data) {
    for (const b of data) this.buf.push(b);
    this._poll();
  }

  _poll() {
    for (;;) {
      let idx = -1;
      for (let i = 0; i < this.buf.length - 1; i++) {
        if (this.buf[i] === 0xAA && this.buf[i + 1] === 0x55) { idx = i; break; }
      }
      if (idx < 0) {
        return;                       // 无帧头（含"只有 AA"等待下一字节）→ 返回
      }
      this.buf = this.buf.slice(idx); // 帧头前字节按噪声丢弃
      if (this.buf.length < HEADER_SIZE) return;
      const n = this.buf[6] | (this.buf[7] << 8);
      if (n > MAX_PAYLOAD) {
        this.buf = this.buf.slice(1); // 错误 LEN：丢弃到下一个可能的帧头
        continue;
      }
      const total = HEADER_SIZE + n + 2;
      if (this.buf.length < total) return;   // 半帧，等待后续
      const frame = this.buf.slice(0, total);
      this.buf = this.buf.slice(total);      // 消费该帧（无论校验成败）
      const crcIn = frame[8 + n] | (frame[9 + n] << 8);
      if (frame[2] !== VERSION) {
        this.events.push(['ack', ACK_FORMAT]);
      } else if (crc16(frame.slice(2, 8 + n)) !== crcIn) {
        this.events.push(['ack', ACK_CRC]);
      } else {
        this.events.push([frame, null]);
      }
    }
  }
}

// =====================================================================
// 测试 1：固定测试向量
// =====================================================================
function test_vectors() {
  console.log('== 固定测试向量 ==');
  const joints_um = [100000, 200000, 300000];
  const joints_urad = [20, 20, 45].map((v) => pround(v * 1e6 * 3.14159265358979 / 180.0));
  const f = make_target(0, joints_um, joints_urad, 0x1234);
  check('TARGET 总长 == 10+28 = 38', f.length === 38, hex(f));
  check('TARGET SOF/VER/CMD', f[0] === 0xAA && f[1] === 0x55 && f[2] === VERSION && f[3] === CMD_TARGET);
  check('TARGET SEQ @4..5 LE', (f[4] | (f[5] << 8)) === 0x1234);
  check('TARGET LEN @6..7 == 28', (f[6] | (f[7] << 8)) === 28);
  check('TARGET 关节1 从 f[12] 起', Buffer.from(f.slice(12, 16)).readInt32LE(0) === joints_um[0]);
  check('TARGET 关节4 从 f[24] 起', Buffer.from(f.slice(24, 28)).readInt32LE(0) === joints_urad[0]);
  check('TARGET CRC @[36..37] 低字节在前',
    f[36] === (crc16(f.slice(2, 36)) & 0xFF) && f[37] === (crc16(f.slice(2, 36)) >> 8));

  const a = append_crc([0xAA, 0x55, VERSION, RESP_ACK, 0x34, 0x12, 4, 0,
    0x34, 0x12, CMD_TARGET, ACK_OK]);
  check('ACK 总长 == 10+4 = 14', a.length === 14, hex(a));
  check('ACK LEN @6..7 == 4', (a[6] | (a[7] << 8)) === 4);
  check('ACK payload seq == 请求 seq', (a[8] | (a[9] << 8)) === 0x1234);
  check('ACK payload cmd/result', a[10] === CMD_TARGET && a[11] === ACK_OK);
  check('ACK CRC @[12..13] 覆盖 10 字节',
    a[12] === (crc16(a.slice(2, 12)) & 0xFF) && a[13] === (crc16(a.slice(2, 12)) >> 8));

  const pos = new Array(12).fill(0);
  // seq, state, fault, valid, enabled, axis_status(uint32)
  let p = [7, 0, STATE_ENABLED, 0, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00];
  for (const v of pos) p = p.concat(i32le(v));
  if (p.length !== 60) throw new Error('STATE payload must be 60 bytes, got ' + p.length);
  const st = append_crc(cat([0xAA, 0x55, VERSION, RESP_STATE, 7, 0, 60, 0], p));
  check('STATE 总长 == 10+60 = 70', st.length === 70, hex(st));
  check('STATE LEN @6..7 == 60', (st[6] | (st[7] << 8)) === 60);
  check('STATE CRC 覆盖 66 字节 @[68..69]',
    st[68] === (crc16(st.slice(2, 68)) & 0xFF) && st[69] === (crc16(st.slice(2, 68)) >> 8));
}

// =====================================================================
// 测试 2：16 种解析场景
// =====================================================================
function test_stream_parser() {
  console.log('== 帧解析场景 ==');
  const f = make_target(0, [100000, 200000, 300000], [174533, 349066, 785398], 5);
  const n_ok = (p) => p.events.filter((e) => e[0] !== 'ack').length;
  const n_ack = (p) => p.events.filter((e) => e[0] === 'ack').length;

  const run = (name, chunks, exp_ok, exp_ack) => {
    const p = new StreamParser();
    for (const c of chunks) p.feed(c);
    check(name, n_ok(p) === exp_ok && n_ack(p) === exp_ack,
      `expected ok=${exp_ok} ack=${exp_ack}, got ok=${n_ok(p)} ack=${n_ack(p)}`);
  };

  const bad = f.map((b, i) => (i === 36 || i === 37) ? (b ^ 0xFF) : b);  // CRC 损坏

  run('1 一次完整 TARGET', [f], 1, 0);
  run('2 一帧拆成两次', [f.slice(0, 20), f.slice(20)], 1, 0);
  run('3 一帧逐字节', f.map((b) => [b]), 1, 0);
  run('4 两帧粘包', [cat(f, f)], 2, 0);
  run('5 三帧粘包', [cat(f, f, f)], 3, 0);
  // 帧 1 完整（含 CRC）+ 帧 2 开头在同一批到达：帧 1 正常解析，帧 2 半帧等待；
  // 补足帧 2 剩余字节后完成解析。
  let p = new StreamParser();
  p.feed(cat(f, f.slice(0, 10)));
  check('6 帧1完整+帧2开头同时到达', n_ok(p) === 1 && n_ack(p) === 0,
    `ok=${n_ok(p)} ack=${n_ack(p)}`);
  p.feed(f.slice(10));
  check('6b 补足帧2剩余字节 → 两帧均解析', n_ok(p) === 2 && n_ack(p) === 0,
    `ok=${n_ok(p)} ack=${n_ack(p)}`);
  run('7 帧头前随机噪声', [cat([0x01, 0x02, 0x03], f)], 1, 0);
  run('8 CRC 错误帧', [bad], 0, 1);
  // LEN 错误：帧本身 CRC 通过，进入命令分发后按 BAD_LEN 处理（本层只统计解析事件；
  // 分发层结果见 test_dispatch）
  run('9 LEN 小于要求（TARGET LEN=20）', [make_frame(CMD_TARGET, 1, new Array(20).fill(0))], 1, 0);
  run('10 LEN 大于要求（TARGET LEN=40）', [make_frame(CMD_TARGET, 1, new Array(40).fill(0))], 1, 0);
  run('11 LEN 超过最大值（60）', [make_frame(CMD_TARGET, 1, new Array(61).fill(0))], 0, 0);
  run('12 不支持的 VERSION', [cat([0xAA, 0x55, 0x02], f.slice(3))], 0, 1);
  run('13 未知 CMD', [make_frame(0x7F, 1, [])], 1, 0);
  // 半帧：feed 前 20 字节后不再 feed，不产生事件（等待后续数据）
  p = new StreamParser();
  p.feed(f.slice(0, 20));
  check('14 半帧长时间无后续数据 → 无事件', n_ok(p) === 0 && n_ack(p) === 0);
  // 溢出恢复：大量噪声（含 AA）后跟合法帧
  run('15 缓冲溢出后恢复', [cat(new Array(300).fill(0xAA), f)], 1, 0);
  run('16 CRC 错误帧后紧跟合法帧', [cat(bad, f)], 1, 1);
}

// =====================================================================
// 测试 3：命令分发层（LEN 精确匹配 + 状态检查）
// =====================================================================
function test_dispatch() {
  console.log('== 命令分发层 ==');
  // 与 Aimotor_HandleBinaryFrame 相同的 LEN/arm 检查逻辑
  const dispatch = (cmd, payload) => {
    if (cmd === CMD_HELLO) return payload.length === 0 ? ACK_OK : ACK_BAD_LEN;
    if (cmd === CMD_ENABLE) return payload.length === 0 ? ACK_OK : ACK_BAD_LEN;
    if (cmd === CMD_STOP) {
      if (payload.length !== 1) return ACK_BAD_LEN;
      // arm 字段仅为兼容保留；固件忽略其值，STOP 始终覆盖双臂 12 轴。
      return ACK_OK;
    }
    if (cmd === CMD_GET_STATE) return payload.length === 0 ? ACK_OK : ACK_BAD_LEN;
    if (cmd === CMD_DISABLE) return payload.length === 0 ? ACK_OK : ACK_BAD_LEN;
    if (cmd === CMD_TARGET) {
      if (payload.length !== 28) return ACK_BAD_LEN;
      return ACK_OK;
    }
    return ACK_UNKNOWN_CMD;
  };

  check('TARGET LEN=20 → BAD_LEN(0x04)', dispatch(CMD_TARGET, new Array(20).fill(0)) === ACK_BAD_LEN);
  check('TARGET LEN=28 → OK', dispatch(CMD_TARGET, new Array(28).fill(0)) === ACK_OK);
  check('STOP arm=2（兼容字段任意值）→ OK', dispatch(CMD_STOP, [2]) === ACK_OK);
  check('STOP arm=255（兼容字段任意值）→ OK', dispatch(CMD_STOP, [255]) === ACK_OK);
  check('STOP arm=1 → OK', dispatch(CMD_STOP, [1]) === ACK_OK);
  check('ENABLE 带 1 字节 payload → BAD_LEN', dispatch(CMD_ENABLE, [0]) === ACK_BAD_LEN);
  check('未知 CMD → UNKNOWN(0x03)', dispatch(0x7F, []) === ACK_UNKNOWN_CMD);
}

// =====================================================================
// 测试 4：换算一致性
// =====================================================================
function test_conversion() {
  console.log('== 换算一致性（文本 vs 二进制）==');
  for (const [j5, j6] of [[20, 45], [-30, 15], [0, 0], [90, -60], [7, -7]]) {
    for (const side of [0, 1]) {
      const [m5_text, m6_text] = fk(side, 0, j5 * 1000, j6 * 1000).slice(1);
      const urad5 = pround(j5 * Math.PI / 180.0 * 1e6);
      const urad6 = pround(j6 * Math.PI / 180.0 * 1e6);
      const [m5_bin, m6_bin] = fk(side, 0, urad_to_deg1000(urad5), urad_to_deg1000(urad6)).slice(1);
      check(`J5/J6 文本==二进制 (side=${side} J5=${j5} J6=${j6})`,
        m5_text === m5_bin && m6_text === m6_bin,
        `text=(${m5_text},${m6_text}) bin=(${m5_bin},${m6_bin})`);
      if (side === 0 && j5 === 20) {
        check('J5 20° → 33333 计数（含 5/3）', m5_text === 33333, String(m5_text));
      }
    }
  }
  check('右臂 J5 取负', fk(1, 0, 20000, 0)[1] === -33333);
  check('J6 耦合 M6 = J6*20/9 - M5（J5=0）', fk(0, 0, 0, 45000)[2] === 100000);
  // 正/逆换算回环（与固件 MW_ReadbackUrad 单位一致，deg×1000）：
  //   M5=33333 → J5_deg1000 = 33333*3/5 = 19999（±1 量化），≈ 20°
  //   M6=66667 → J6_deg1000 = (66667+33333)*9/20 = 45000，= 45°
  const [, m5, m6] = fk(0, 0, 20000, 45000);
  const j5_inv_d1000 = pdiv(m5 * 3, 5);
  const j6_inv_d1000 = pdiv((m6 + m5) * 9, 20);
  check('反馈 J5 逆换算≈20°（±1 deg1000 量化）', Math.abs(j5_inv_d1000 - 20000) <= 1,
    `j5_inv=${j5_inv_d1000}`);
  check('反馈 J6 逆换算 45°（deg×1000）', j6_inv_d1000 === 45000,
    `j6_inv=${j6_inv_d1000}`);
  check('AI 越界拒绝（J1=50m）', ai_pulse(50000000, 0) > AIMOTOR_POSITION_MAX);
  check('MW 越界拒绝（J5=40000°）', MW_DEG1000_LIMIT < 40000 * 1000);
}

// =====================================================================
// 测试 5：控制状态机
// =====================================================================
function test_state_machine() {
  console.log('== 控制状态机 ==');
  const target_allowed = (s) => s === STATE_ENABLED;
  check('DISABLED 拒绝 TARGET', !target_allowed(STATE_DISABLED));
  check('ENABLED 允许 TARGET', target_allowed(STATE_ENABLED));
  check('STOPPED 拒绝 TARGET', !target_allowed(STATE_STOPPED));
  check('FAULT 拒绝 TARGET', !target_allowed(STATE_FAULT));
}

// =====================================================================
// 人工验收步骤
// =====================================================================
function manual_steps() {
  console.log(`
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
`);
}

function main() {
  console.log('=== STM32 二进制协议主机侧测试（Node 移植自 protocol_test.py）===');
  test_vectors();
  test_stream_parser();
  test_dispatch();
  test_conversion();
  test_state_machine();
  manual_steps();
  console.log(`=== 结果：${_passed} passed, ${_failed} failed ===`);
  return _failed === 0 ? 0 : 1;
}

process.exit(main());
