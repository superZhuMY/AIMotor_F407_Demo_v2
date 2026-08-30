# 双臂机械臂 — 上位机串口通信 API 文档 v1.0

## 1. 串口参数

| 参数 | 值 |
|------|-----|
| 波特率 | **115200** |
| 数据位 | 8 |
| 校验位 | None |
| 停止位 | **1** |
| 换行符 | `\r\n` (CR+LF) |
| 编码 | ASCII 文本 |

## 2. 命令规则

所有命令以 `\r\n` 结尾。不区分大小写。

---

## 3. 关节映射

双臂各有 6 个关节 J1~J6。左臂=L，右臂=R。

```
┌───────────────────────────────────────────────┐
│  左臂 (L)              右臂 (R)  (镜像对称)     │
│                                               │
│  总线 #1 (USART2): B1   总线 #2 (USART3): B2    │
│  ┌─────┬────────┬───┐  ┌─────┬────────┬───┐   │
│  │关节 │ 电机   │轴 │  │关节 │ 电机   │轴 │   │
│  ├─────┼────────┼───┤  ├─────┼────────┼───┤   │
│  │ J1  │ B1 M1  │ X │  │ J1  │ B2 M1  │ X │   │
│  │ J2  │ B1 M2  │ Z │  │ J2  │ B2 M2  │ Z │   │
│  │ J3  │ B1 M3  │ Y │  │ J3  │ B2 M3  │ Y │   │
│  └─────┴────────┴───┘  └─────┴────────┴───┘   │
│                                               │
│  总线 #3 (USART6): L    总线 #4 (UART5):  R     │
│  ┌─────┬────────┬───┐  ┌─────┬────────┬───┐   │
│  │ J4  │  ID1   │Y旋│  │ J4  │  ID1   │Y旋│   │
│  │ J5  │  ID2   │X旋│  │ J5  │  ID2   │X旋│   │
│  │ J6  │  ID3   │Z旋│  │ J6  │  ID3   │Z旋│   │
│  └─────┴────────┴───┘  └─────┴────────┴───┘   │
└───────────────────────────────────────────────┘

轴说明:
  J1 → X 轴 (左右横移)
  J2 → Z 轴 (上下升降)
  J3 → Y 轴 (前后伸缩)
  J4 → Y 轴旋转 (沿Y转动, 独立关节)
  J5 → X 轴旋转 (沿X转动, ←与J6耦合)
  J6 → Z 轴旋转 (沿Z转动, ←与J5耦合)

耦合关系: J5/J6 有机械耦合，MCU 内部自动解算。
         上位机只需发目标关节角度，无需关心电机关系。
```

---

## 4. 命令列表

### 4.1 平移关节 J1~J3（AI 电机 — Modbus RTU）

| 命令 | 说明 | 示例 |
|------|------|------|
| `B{1/2} M{1/2/3} P{位置} S{速度}` | 移动关节到目标位置 | `B1 M1 P10000 S300` |
| `B{1/2} M{1/2/3} Q` | 查询关节当前位置 | `B1 M1 Q` |
| `B{1/2} M{1/2/3} HOME` | 关节独立回零 | `B1 M1 HOME` |
| `B{1/2} STOP` | 全局停止双臂 12 轴（总线号仅兼容保留） | `B1 STOP` |

| 参数 | 含义 | 单位 | 范围 |
|------|------|------|------|
| P | 目标位置 | 脉冲 | -9999999 ~ 9999999 |
| S | 运行速度 | rpm | 0 ~ 6000 |

### 4.2 旋转关节 J4~J6（MW 电机 — 私有协议 0x3E）

| 命令 | 说明 | 示例 |
|------|------|------|
| `L/R J4{角度} J5{角度} J6{角度} S{速度}` | 旋转关节角度控制 | `L J4 20 J5 30 J6 45 S180` |
| `L/R Q` | 查询旋转关节当前角度 | `L Q` |
| `L/R HOME` | 该臂全部旋转关节回零 | `L HOME` |
| `L/R J4 HOME` | 指定旋转关节回零 | `L J4 HOME` |
| `L/R STOP` | 全局停止双臂 12 轴（L/R 仅兼容保留） | `L STOP` |

| 参数 | 含义 | 单位 | 范围 |
|------|------|------|------|
| J5 | X轴旋转角度 | 度 (°) | -36000 ~ 36000 |
| J6 | Z轴旋转角度 | 度 (°) | -36000 ~ 36000 |
| S | 旋转速度 | 度/秒 (°/s) | 0 ~ 36000 |

> J4/J5/J6 可组合发送，如只发 `L J4 90 S100` 仅控制 J4。

### 4.3 全局命令

| 命令 | 说明 |
|------|------|
| `ALL STOP` | 紧急停止所有电机 |

---

## 5. 响应格式

### 5.1 移动命令

```
→ B1 M1 P10000 S300
← OK B1 M1 P10000 S300

→ L J4 20 J5 30 J6 45 S180
← OK L J4=20 J5=30 J6=45 S=180
```

### 5.2 查询命令

```
→ B1 M1 Q
← POS B1 M1 12345
      │   │   │   └─ 当前位置(脉冲)
      │   │   └─ 关节号
      │   └─ 总线号
      └─ 固定前缀

→ L Q
← POS L J5=30 J6=45
      │   │      └─ J6当前角度(度)
      │   └─ J5当前角度(度)
      └─ 左臂
```

### 5.3 停止命令

```
→ B1 STOP
← OK STOP: ACCEPTED

→ ALL STOP
← OK STOP: ACCEPTED
```

---

## 6. 通信流程示例

上位机每隔约 150ms 轮询一次所有 12 个关节：

```
--- 左臂平移 (L J1~J3) ---
→ B1 M1 Q
← POS B1 M1 12345      ← J1 (X轴) 当前位置

→ B1 M2 Q
← POS B1 M2 5000       ← J2 (Z轴) 当前位置

→ B1 M3 Q
← POS B1 M3 -2000      ← J3 (Y轴) 当前位置

--- 右臂平移 (R J1~J3) ---
→ B2 M1 Q
← POS B2 M1 11234

→ B2 M2 Q
← POS B2 M2 4800

→ B2 M3 Q
← POS B2 M3 -1800

--- 左臂旋转 (L J4~J6) ---
→ L Q
← POS L J5=30 J6=45    ← J5/J6当前角度(度)

--- 右臂旋转 (R J4~J6) ---
→ R Q
← POS R J5=-30 J6=-45

... 循环 ...
```

用户点击按钮时中断轮询，发送移动命令：

```
→ B1 M1 P15000 S300
← OK B1 M1 P15000 S300     ← 确认收到
... 恢复轮询 ...
```

---

## 7. HTML/JS 实现要点

```javascript
// 连接串口 (仅 Chrome/Edge 支持)
const port = await navigator.serial.requestPort();
await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: "none" });

// 发送命令
async function sendCmd(cmd) {
    const encoder = new TextEncoder();
    const writer = port.writable.getWriter();
    await writer.write(encoder.encode(cmd + "\r\n"));
    writer.releaseLock();
}

// 轮询查询所有关节位置
const QUERY_SEQUENCE = [
    "B1 M1 Q", "B1 M2 Q", "B1 M3 Q",   // 左臂 J1~J3
    "B2 M1 Q", "B2 M2 Q", "B2 M3 Q",   // 右臂 J1~J3
    "L Q", "R Q",                        // 左右旋转 J4~J6
];

let queryIndex = 0;
setInterval(async () => {
    await sendCmd(QUERY_SEQUENCE[queryIndex]);
    queryIndex = (queryIndex + 1) % QUERY_SEQUENCE.length;
}, 150);  // 每150ms发一条, 1.8秒完成一轮
```

---

## 8. 二进制协议附录（ROS 2 串口桥接使用）

上位机除 ASCII 文本命令外，也可使用 `AA 55` 开头的二进制帧。帧格式、
命令码、错误码、状态机与看门狗规则以 `docs/README_J456_DRIVER.md` §6 为唯一
权威定义；本节为 ROS 2 桥接节点实现要点。

### 8.1 帧格式（固定）

```text
AA 55 | VERSION(1)=0x01 | CMD(1) | SEQ(2,LE) | LEN(2,LE) | PAYLOAD(LEN) | CRC16(2,LE)
byte[0..1] | [2] | [3] | [4..5] | [6..7] | [8..8+LEN-1] | [8+LEN..9+LEN]
```

- 总帧长 `10 + LEN`；CRC 覆盖 `VERSION → PAYLOAD 末尾`（`6+LEN` 字节），
  Modbus CRC-16（poly 0xA001，初值 0xFFFF），低字节在前。

### 8.2 命令与响应

| CMD | 名称 | LEN | PAYLOAD |
|---|---|---|---|
| 0x01 | HELLO | 0 | — |
| 0x02 | ENABLE | 0 | — |
| 0x03 | STOP | 1 | arm(兼容保留，值任意并忽略；始终全局停止 12 轴) |
| 0x04 | GET_STATE | 0 | — |
| 0x05 | DISABLE | 0 | — |
| 0x10 | TARGET | 28 | arm(1)+mode(1)+flags(2)+J1..J3(int32 µm)+J4..J6(int32 µrad)，关节从 PAYLOAD 偏移 4 起 |
| 0x80 | ACK | 4 | seq(2,LE)+cmd(1)+result(1) |
| 0x81 | STATE | 60 | 见 §8.4 |

错误码（result）：0=成功，1=CRC，2=格式/版本，3=未知CMD，4=长度错，
5=参数越界，6=状态不允许，7=TARGET 无效arm，8=通信超时，9=控制序列忙（CTRL_BUSY），
10=控制命令部分/全部电机失败（CTRL_FAILED）。控制类命令（ENABLE/STOP/DISABLE）
的 ACK 在异步序列完成后回发，`0x00`=序列完成且全部电机确认，`0x0A`=存在失败轴。

### 8.3 关节换算（文本/二进制唯一基准）

```text
J1~J3: pulse = µm×10/4、µm×10000/475、µm×10/2（限幅 ±10,000,000）
J4:    M4 = J4(deg)×1000
J5:    M5 = J5(deg)×1000×5/3   （右臂取负）
J6:    M6 = J6(deg)×1000×20/9 − M5
```

状态反馈的逆换算与之对应（J5 反含 ÷5/3 与右臂取反，J6 反含 +M5 与 ÷20/9）。

### 8.4 STATE 帧（LEN=60，总长 70）

```text
seq(2) | control_state(1) | fault(1) | valid(2) | enabled(2) | axis_status(4)
12×int32(LE)：L_J1µm, L_J2µm, L_J3µm, L_J4..6µrad, R_J1µm..R_J6µrad
```

- `control_state`：0=DISABLED，1=ENABLED，2=STOPPED，3=FAULT。
- `fault` 位：bit0=通信超时，bit1=接收溢出，bit2=ENABLE 失败，bit3=未确认停止，bit4=内部错误。
- `valid` bit0..11 按 `side*6+joint`（L_J1..L_J6, R_J1..R_J6），1=有真实回读；
  无回读轴位置为 0 且对应位为 0，不伪造。
- `enabled` bit0..11 同序，1=该轴已确认使能（ENABLE 序列回包确认）。
- `axis_status` **uint32**，每轴 2bit（顺序同 valid）：bit0=online（真实回读），
  bit1=fault；共 24bit，右臂 J3..J6（bit16..22）不丢失。
- 位置为关节侧单位（µm / µrad），LE int32，由驱动器真实回读换算。

### 8.5 控制状态机与看门狗

- 上电默认 DISABLED；MCU 复位后异步执行全轴安全停止（AI Servo Off + MW 0x80），
  序列完成前状态保持 DISABLED；TARGET 仅在 ENABLED 状态允许，TARGET 不隐式使能。
- ENABLE：异步序列 AI 6 台 ServoOn + MW 6 台 0x88，全部确认成功才进入 ENABLED；
  DISABLE：先停 12 轴再 AI Servo Off。
- 已处于 ENABLED 且 12 轴均确认使能、无轴故障时，重复 ENABLE 为幂等操作：
  立即 ACK 0x00 并刷新看门狗，不重跑使能序列；状态不一致或存在轴故障时回 0x0A。
- 普通运动事务通信故障锁存到下一次成功 ENABLE；故障臂的新 TARGET 回 0x0A，
  不得通过下一条 TARGET 自动清故障重试。建议先 STOP/DISABLE，再 ENABLE 恢复。
- 通信看门狗 250ms：ENABLED 状态超过 250ms 无有效命令 → 立即 STOPPED、
  清 pending、启动全局停止序列；需显式 ENABLE 恢复。
- **看门狗刷新表**：HELLO 成功→**固定不刷新**；ENABLE/TARGET/GET_STATE/STOP/DISABLE
  成功接受→刷新；CRC 错/非法 LEN/未知 CMD/arm 错/越界/状态拒绝/序列忙/失败→不刷新。
  有效文本命令一律不刷新。

### 8.6 版本记录

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.0 | 2026-07-16 | 初始版本，L/R 双臂 J1~J6 共 12 关节映射 |
| v1.1 | 2026-08-09 | 统一二进制帧格式（帧头 8 字节，payload 从 byte[8] 起）；修复 ACK LEN=4；新增 ENABLE/DISABLE/GET_STATE/HELLO、STATE 反馈帧；修复二进制 J5 漏 5/3；上电默认 DISABLED；250ms 通信看门狗；DMA 流解析（拆包/粘包/CRC 重同步） |
| v1.2 | 2026-08-09 | STATE 帧 LEN=60（新增 enabled 位图、axis_status 升 uint32）；STOP 改为全局安全停止双臂 12 轴（arm 字段仅兼容保留并忽略）；ENABLE/STOP/DISABLE 经异步控制序列执行（回包确认，ACK 在序列完成后回发）；FAULT 不可恢复位拒绝 ENABLE；部分失败自动回滚；文本运动命令默认关闭 |
| v1.3 | 2026-08-09 | 重复 ENABLE 改为立即幂等 ACK，消除看门狗盲区；运动通信故障锁存；文本 STOP 可抢占控制序列；B TEST 永久禁用，GO 改为异步状态机；修正主机测试与自检 |
