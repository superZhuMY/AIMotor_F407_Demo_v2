# J1～J6 电机底层驱动改动说明

本版本针对 STM32F407 下位机，统一整理了两类电机的 RS485 驱动、J5/J6
耦合换算、回包校验和串口异常恢复。到位判断代码已经保留但默认关闭，当前
先用于验证动作和通信稳定性，没有改变上位机原有的角度单位和 ROS 兼容换算。

## 1. 总线和波特率

| 轴 | 协议 | MCU 串口 | 波特率/格式 | 备注 |
|---|---|---|---|---|
| J1～J3 | AI Motor Modbus RTU | USART2（左）、USART3（右） | **115200, 8N2** | 3 个 ID 轮询 |
| J4～J6 | 慕纬度私有协议 | USART6（左）、UART5（右） | **115200, 8N1** | ID 4/5/6 |
| 上位机 | 文本命令 | USART1 | 115200, 8N1 | DMA 空闲接收 |

所有 RS485 模块都自动控制收发方向，因此本工程没有额外 DE/RE 切换引脚。
115200 是本项目的明确硬件配置；AI 电机的手册默认值为 57600，烧录前必须
确认每个电机已经通过参数 H0C-02 配置为 115200（值 6），修改后按手册要求
保存并重新上电。ROS 仓库中的旧节点仍写着 57600，不能作为本下位机当前配置。

## 2. ROS 兼容的角度和耦合换算

协议内部仍使用 ROS 当前代码的 `1000 count/degree`：

```text
左臂: M5 = J5 * 1000 * 5/3
      M6 = J6 * 1000 * 20/9 - M5

右臂: M5 = -J5 * 1000 * 5/3
      M6 = J6 * 1000 * 20/9 - M5
```

反馈换算为：

```text
J5 = M5 / 1000 / (5/3)       （右臂再取负）
J6 = (M6 + M5) / 1000 / (20/9)
```

J5/J6 是一个事务，不能分别独立下发。新命令进入 pending 缓冲，当前事务
完成一次 “写 M5 → 写 M6 → 读 M5 → 读 M6” 后才锁存下一组目标；这样不会在
一帧尚未完成时被新命令覆盖。J4 仍独立控制。协议中的 A3 位置帧不携带速度，
上位机 `S` 参数仅为兼容旧命令而保留，不会改变电机速度。

## 3. AI Motor（J1～J3）到位判断

本版本保留了位置反馈和位置偏差反馈代码，但**当前测试阶段不启用到位判定**：

| 寄存器 | 含义 | 本工程用途 |
|---|---|---|
| H0B-07 | 绝对位置计数器，指令单位 | 记录当前位置 |
| H0B-15 | 编码器位置偏差计数器 | 软件到位判据 |
| H05-21 | 定位完成阈值，默认 92 指令单位 | 手册中的 COIN 到位阈值 |
| H0A-10 | 位置偏差故障阈值，默认 1048576 编码单位 | 故障阈值，不是到位阈值 |

手册给出的电子齿轮默认值为 H05-07/H05-09 = 131072/1000，因此：

```text
ceil(92 × 131072 / 1000) = 12059 编码单位
```

换算后的 `AIMOTOR_POS_ERROR_TOLERANCE_ENCODER` 默认值为 12059，但暂不会参与
状态跳转；H0B-15 仍可被读取和缓存，便于后续重新启用。重新启用时，将
`Core/Inc/aimotor.h` 中的 `AIMOTOR_ENABLE_ARRIVAL_CHECK` 改为 `1`，再根据
现场电子齿轮和 H05-21 参数确认阈值。

## 4. 可靠性改动

- 严格检查帧头、站号、功能码、长度和校验；Modbus 写停止/写位置/触发帧也必须
  收到匹配回包才进入下一步。
- 每条总线一次只允许一个未完成请求；超时重试有上限，不再无限阻塞等待 TC。
- J1～J3 使用 5 ms 调度周期轮转，单个电机理论轮询间隔约 15 ms；建议先以
  20 Hz 发送目标，确认总线负载后再逐步提高。
- J5/J6 使用响应驱动的耦合状态机，响应超时 30 ms；当前完成一轮
  “写 M5 → 写 M6 → 读 M5 → 读 M6”后即结束本次事务，不等待到位。
  后续启用到位判断时，将 `Core/Inc/mwmotor.h` 中的
  `MW_ENABLE_ARRIVAL_CHECK` 改为 `1`，并现场标定 `MW_ARRIVAL_TOLERANCE`。
- 新命令不会打断当前串口事务；`STOP`/`ALL STOP` 会清空 pending、取消耦合任务，
  并向相关电机发送 0x80。
- 增加 UART 错误回调：清除 ORE/NE/FE/PE，终止并重新启动 DMA 接收，避免手动复位
  STM32 后才能恢复。
- 二进制协议带完整控制状态机与 250ms 通信看门狗（见 §6.6）；上电默认 DISABLED，
  复位不会自动使能电机。

## 5. 上位机调试命令

命令以 ASCII 文本通过 USART1 发送，建议以 `\r\n` 结尾：

```text
L J4 10 J5 20 J6 -15       # 左臂目标角度（度）
R J5 20 J6 -15             # 右臂 J5/J6 耦合目标
L J4 HOME                  # J4 回零
R J5 HOME                  # J5/J6 一起回零
L Q                        # 查询缓存的 J4/J5/J6 角度
ALL STOP                   # 双臂 12 轴全部停止
ALL ENABLE                 # 全部 12 电机使能（AI ServoOn + MW 0x88）
ALL DISABLE                # 全部停止并禁用
```

未出现在命令中的关节保持上一次目标；不要发送空的 J5/J6 帧。

> **文本运动命令开关（`AIMOTOR_TEXT_MOTION_ENABLED`，默认 0=发布配置）**：
> 发布/真机版本默认关闭所有会引起运动或使能的文本命令
> （`B M P`、`B M HOME`、`B M GO`、`B M EN`、`B TEST`、`ALL ENABLE`、
> `L/R J4 J5 J6`、`L/R HOME`），返回 `ERR TEXT MOTION DISABLED`。只读命令
> （`B M Q`、`L/R Q`）与安全命令（`B STOP`、`L/R STOP`、`ALL STOP`、
> `ALL DISABLE`）始终可用。置 1 时文本运动命令必须经过与二进制相同的控制状态检查
> （非 ENABLED 拒绝）；单轴 `B M EN` 已彻底移除（避免无回包确认直接写 enabled，
> 统一使用二进制 ENABLE 或 `ALL ENABLE`）。`B TEST` 即使在调试配置也永久禁用；
> `B M GO` 交给生产异步状态机执行，不使用 `HAL_Delay()`。
> **文本命令不刷新通信看门狗**（仅二进制有效命令刷新）。

```text
ALL ENABLE                 # 全部 12 电机使能（异步序列，需 TEXT_MOTION_ENABLED=1）
ALL DISABLE                # 全部停止并禁用（异步序列）
```

文本 `B STOP`/`L/R STOP`/`ALL STOP` 统一走异步**全局安全停止序列**（双臂 12 轴，
逐台发送并回包确认，非阻塞），且可抢占 ENABLE/DISABLE/回滚序列，回显
`OK STOP: ACCEPTED`；文本
`ALL ENABLE`/`ALL DISABLE` 同样通过异步控制序列执行（回显 ACCEPTED），
序列完成的真实结果仅由二进制 ACK 携带。

## 6. USART1 二进制帧（新增，ASCII 仍保留）

STM32 已支持以 `AA 55` 开头的二进制帧；首字节不是 `AA 55` 时仍按原 ASCII
命令处理，便于串口助手继续调试。

### 6.1 固定帧格式

```text
AA 55 | VERSION(1) | CMD(1) | SEQ(uint16,LE) | LEN(uint16,LE)
      | PAYLOAD(LEN) | CRC16(uint16,LE)
```

| 字节索引 | 字段 | 说明 |
|---|---|---|
| 0..1 | `AA 55` | 帧头 |
| 2 | VERSION | `0x01` |
| 3 | CMD | 命令码（下表） |
| 4..5 | SEQ | 小端 uint16，请求序列号 |
| 6..7 | LEN | 小端 uint16，PAYLOAD 长度 |
| 8..8+LEN-1 | PAYLOAD | 负载 |
| 8+LEN..9+LEN | CRC16 | 小端，Modbus CRC-16 |

- 固定帧头 8 字节；`总帧长 = 10 + LEN`；CRC 位于 `8+LEN`、`9+LEN`。
- CRC 算法：Modbus CRC-16（poly `0xA001` 反射，初值 `0xFFFF`），
  计算范围 `VERSION(byte2) → PAYLOAD 末尾`，即 `6 + LEN` 字节，线上低字节在前。
- 多字节均为小端（LE）。不允许任何一端的长度/CRC/偏移与此表不一致。

### 6.2 命令码（CMD）

| CMD | 名称 | LEN | PAYLOAD |
|---|---|---|---|
| `0x01` | HELLO | 0 | —（回 ACK result=0） |
| `0x02` | ENABLE | 0 | —（全局，全部 12 电机） |
| `0x03` | STOP | 1 | `arm(uint8)` 仅兼容保留，值任意并忽略；始终全局停止双臂 12 轴 |
| `0x04` | GET_STATE | 0 | —（回 STATE 帧，带请求 SEQ） |
| `0x05` | DISABLE | 0 | —（全局停止+禁用） |
| `0x10` | TARGET | 28 | 见 6.3 |
| `0x80` | ACK（响应） | 4 | `seq(uint16,LE)+cmd(uint8)+result(uint8)` |
| `0x81` | STATE（响应） | 60 | 见 6.5 |

### 6.3 TARGET 帧（CMD=0x10，LEN=28）

```text
PAYLOAD: arm(uint8) | mode(uint8) | flags(uint16,LE) | 6×int32(LE)
         [8]        [9]          [10..11]            [12..35]
```

- `arm`：0=左臂，1=右臂。`mode`/`flags` 预留，当前忽略。
- 关节从 PAYLOAD 偏移 4（帧内 `f[12]`）开始：
  `J1/J2/J3` 为 int32 小端 **µm**；`J4/J5/J6` 为 int32 小端 **µrad**。
- 整帧原子性：LEN=28、arm 合法、控制状态 ENABLED、全部 6 关节换算后
  在安全范围内，才统一下发；任一字段错误整帧拒绝并回错误 ACK，
  不允许“部分关节已执行”。
- 换算（唯一基准，文本/二进制共用同一函数）：
  - J1~J3 µm→脉冲：`×10/4`、`×10000/475`、`×10/2`，结果限幅 ±10,000,000 脉冲；
  - J4：`M4 = J4×1000`；
  - J5：`M5 = J5×1000×5/3`（右臂取负）；
  - J6：`M6 = J6×1000×20/9 − M5`（耦合）。
- `5/3`、`20/9` 与右臂取反只应用一次，位于共享换算函数中。

### 6.4 ACK 帧（CMD=0x80，LEN=4，总长 14）

```text
AA 55 01 80 | SEQ(2)=请求seq | LEN=4 | seq(2,LE)+cmd(1)+result(1) | CRC(2)
byte[0..1]   [2..3]           [6..7]  [8..11]                      [12..13]
```

- ACK 的 SEQ（`f[4..5]`）与 payload 中的 seq（`f[8..9]`）均为请求 SEQ。
- 成功 ACK 仅表示**已受理**（帧解析+状态检查通过），不表示执行完成。

结果码（result）：

| 值 | 含义 |
|---|---|
| `0x00` | 成功 |
| `0x01` | CRC 校验失败 |
| `0x02` | 帧格式/版本错误 |
| `0x03` | 未知 CMD |
| `0x04` | 该 CMD 的 payload 长度错误 |
| `0x05` | 关节参数越界 |
| `0x06` | 当前控制状态不允许（如未 ENABLE 发 TARGET） |
| `0x07` | 无效 arm |
| `0x08` | 通信超时（需重新 ENABLE） |
| `0x09` | 控制序列进行中，命令被拒绝（CTRL_BUSY） |
| `0x0A` | 控制命令部分/全部电机失败（CTRL_FAILED） |

> **ACK 阶段语义**：`0x00` = 命令已接受/序列已完成且全部电机确认成功；
> `0x0A` = 序列完成但存在失败轴（STATE 中该轴 fault 位置位）；
> 控制类命令（ENABLE/STOP/DISABLE）的 ACK 在异步序列完成后才回发，
> 不代表"已收到"而是代表"执行结果"。

### 6.5 STATE 帧（CMD=0x81，LEN=60，总长 70）

```text
PAYLOAD: seq(2,LE) | control_state(1) | fault(1) | valid(2,LE)
         enabled(2,LE) | axis_status(4,LE)
         12×int32(LE)：L_J1µm, L_J2µm, L_J3µm, L_J4..6µrad,
                       R_J1µm, R_J2µm, R_J3µm, R_J4..6µrad
```

- `control_state`：0=DISABLED，1=ENABLED，2=STOPPED，3=FAULT。
- `fault` 位：bit0=通信超时，bit1=接收溢出，bit2=ENABLE 失败，bit3=未确认停止，bit4=内部错误。
- `valid`：bit0..11 按关节序 `L_J1..L_J6, R_J1..R_J6`（`bit = side*6 + joint`），
  1=该轴有真实回读。无回读的轴位置以 0 填充且对应位清 0，不伪造为真实反馈。
- `enabled`：bit0..11 同序，1=该轴已确认使能（ENABLE 序列 0x88/ServoOn 回包确认）。
- `axis_status`：**uint32**，每轴 2bit，顺序同 `valid`——bit0=online（真实回读），
  bit1=fault。共 24bit，覆盖右臂 J3..J6（bit16..22）。
- 位置均为**关节侧**单位：J1~J3 为 µm，J4~J6 为 µrad；
  来源为驱动器真实回读（AI H0B-07、MW 0x92），经与 TARGET 相反的
  统一逆换算（J5 反向含 `÷5/3` 与右臂取反，J6 反向含 `+M5` 与 `÷20/9`）。

### 6.6 控制状态机与通信看门狗

| 状态 | ENABLE | TARGET | GET_STATE | STOP | DISABLE |
|---|---|---|---|---|---|
| DISABLED（上电默认） | 允许 | **拒绝**（0x06） | 允许 | 允许 | 允许 |
| ENABLED | 允许（幂等） | 允许 | 允许 | 允许 | 允许 |
| STOPPED | 允许 | **拒绝**（0x06） | 允许 | 允许 | 允许 |
| FAULT | 按恢复条件 | **拒绝**（0x06） | 允许 | 允许 | 允许 |

- 上电/复位后默认 `CONTROL_DISABLED`，MCU 完成通信初始化后**异步执行全轴安全
  停止序列**（AI 6 台 Servo Off `0x0303=0` + MW 6 台 `0x80` 关闭），序列完成前
  状态保持 DISABLED；绝不自动使能，绝不恢复复位前目标。直接发 TARGET 返回 0x06。
- ENABLE 为显式命令：异步序列依次向 AI 6 电机发送 Servo On（`0x0303=1`）并向
  MW 6 电机发送 `0x88`，每台等待应答校验；全部确认成功后才进入 ENABLED 并
  回 ACK 0x00。任一轴失败则回 ACK 0x0A 并进入 FAULT，且对**全部 12 轴**执行
  安全停止+禁用回滚（AI StopMotion + MW `0x80` + AI Servo Off）——因为
  "无成功应答"只说明状态未知，不能证明驱动未执行，回滚不遗漏应答丢失的轴。
  TARGET 绝不隐式使能。
- 已经 ENABLED 且 12 轴均确认使能、无轴故障时，重复 ENABLE 是幂等命令：立即
  回 ACK 0x00 并刷新看门狗，不重跑 12 轴序列；若 enabled/fault 状态不一致则回
  0x0A，要求先 STOP/DISABLE，再 ENABLE 恢复。
- AI/MW 普通运动事务重试耗尽后的轴 fault 锁存；故障臂的新 TARGET 回 0x0A，
  不会通过下一条 TARGET 自动清故障并重启。成功 ENABLE 才清除可恢复轴故障。
- DISABLE：接受时立即清除全部旧运动事务（AI 轮转、MW 耦合、J4、空闲读取）
  并进入 DISABLED（非运动过渡状态），随后异步序列先停止 12 轴
  （AI `H03_05=0`、MW `0x80`），再对 AI 执行真实 Servo Off。
- 通信看门狗 `250ms`（`HOST_COMM_WATCHDOG_MS`）：ENABLED 状态下超过
  250ms 未收到**有效命令**立即置 STOPPED（TARGET 即刻被拒）、清空全部待执行
  目标，并启动全局停止序列停 12 轴。此后必须显式 ENABLE 才能恢复；
  仅下一帧 TARGET 不能自动恢复。
  控制序列进行中（ENABLE/STOP/DISABLE/回滚）看门狗不启动新序列；重复 ENABLE
  不创建控制序列，因此不会形成看门狗屏蔽窗口；
  且 STOP/DISABLE 在接受时已进入非运动状态，看门狗不会抢占其安全动作。
- **看门狗刷新表（最终定义）**：

| CMD | 接受成功 | 长度错 | 未知/版本错 | arm 错 | 越界 | 状态拒绝 | 控制序列忙/失败 |
|---|---|---|---|---|---|---|---|
| HELLO | **不刷新** | 不刷新 | 不刷新 | — | — | — | — |
| ENABLE | 刷新 | 不刷新 | 不刷新 | — | — | 不刷新 | 不刷新 |
| TARGET | 刷新 | 不刷新 | 不刷新 | 不刷新 | 不刷新 | 不刷新 | 不刷新 |
| GET_STATE | 刷新 | 不刷新 | 不刷新 | — | — | — | — |
| STOP | 刷新 | 不刷新 | 不刷新 | 不刷新* | — | — | 不刷新 |
| DISABLE | 刷新 | 不刷新 | 不刷新 | — | — | — | 不刷新 |

  *STOP 的 arm 字段仅作兼容保留并明确忽略（LEN=1 校验通过即可，值任意），
  STOP 为全局安全停止双臂 12 轴。
  只有帧结构完整 + CRC 正确 + VERSION 支持 + CMD 已定义 + payload 长度准确 +
  参数合法 + 状态允许 + 命令被接受（或异步序列已启动）的命令才刷新；
  半帧、随机数据、CRC 错、非法 LEN、未知 CMD、越界、状态拒绝、序列忙均不刷新。
  **ENABLE/STOP/DISABLE 的看门狗刷新延迟到序列成功完成时**（失败 0x0A 不刷新；
  序列执行期间由其它有效命令刷新），保证进入 ENABLED 前时间戳新鲜，避免旧时间戳
  立刻触发看门狗。有效**文本**命令一律不刷新看门狗（文本不得成为维持 ROS 2
  连接的方式）。

### 6.7 接收流解析（拆包/粘包）

一次 DMA IDLE 回调收到的字节只追加到 256 字节环形缓冲（`PROTOCOL_RX_BUFFER_SIZE`），
由主循环流解析按以下顺序处理：查 `AA 55` → 帧头前字节按噪声丢弃 →
缓存不足 8 字节等待 → 读 LEN（>`PROTOCOL_MAX_PAYLOAD`=60 报错并跳到下一帧头）→ `total=10+LEN` →
不足等待 → 校验 VERSION/LEN/CRC → 成功分发命令并消费 → 继续解析剩余字节；
CRC 错误帧重同步到下一帧头，不卡死。环形缓冲满时丢弃到下一个帧头并置溢出标志。

### 6.8 新增文本命令与干跑/自检模式

- 新增 `ALL ENABLE` / `ALL DISABLE` 文本命令（见 §5）。
- `AIMOTOR_DRY_RUN` 宏（`Core/Inc/main.h`，默认 0）：置 1 时 **AI 与 MW 两族
  总线的真实发送函数全部被短路**（真实发送计数 `g_dry_run_ai/mw_tx_count` 恒为 0），
  待发送命令由 mock 环形日志记录；参数检查、换算、状态机、ACK/STATE 正常执行。
  安全 STOP/Servo Off 在 DRY_RUN 下同样不发送到总线（由另一个开关决定是否记录，
  当前默认不发送任何电机命令）。编译日志通过 `#pragma message` 打印三个配置宏的
  最终值，防止宏未生效。
- `AIMOTOR_SELF_TEST`（默认 0，需配合 DRY_RUN=1）：main 启动后执行
  `Aimotor_SelfTest()`，**直接调用生产 C 函数**（`Aimotor_HostStreamPoll`、
  `Aimotor_HandleBinaryFrame`、`Aimotor_CommWatchdog`、`CtrlSeqStart/Tick`、
  `MW_ForwardKin`、`MW_ReadbackUrad`、`Aimotor_SendState`、真实发送函数），
  覆盖解析器进度保证、看门狗刷新表、重复 ENABLE 断联、电机安全（ENABLE
  失败→FAULT、全 12 轴回滚真实步骤、重试上限、全局 STOP）、换算与 STATE 位图、
  DRY_RUN 计数，并逐项打印 PASS/FAIL 到 USART1。

## 7. 关键文件

- `Core/Src/aimotor.c` / `Core/Inc/aimotor.h`：J1～J3 Modbus RTU。
- `Core/Src/mwmotor.c` / `Core/Inc/mwmotor.h`：J4～J6 私有协议和 J5/J6 耦合。
- `Core/Src/main.c`：DMA 空闲接收、UART 错误恢复和 5 ms 调度。

## 8. 编译、烧录和现场验证

1. 在 Keil/STM32CubeIDE 中重新编译工程，**不要直接使用压缩包内可能遗留的
   `.hex`/`.bin`**。
2. 确认电机端波特率、校验位、停止位和站号与上表一致。
3. 单轴低角度测试 J1～J4，再测试 `J5/J6` 联动；先用 5～10 Hz 命令。
4. 当前阶段重点观察串口回包和实际动作；H0B-15 只作为缓存数据，不作为到位条件。
   若多次超时，优先检查终端
   电阻、共地、线序和电机参数，不要盲目提高发送频率。
5. 本次修改未在实际电机和示波器上执行验证；`MW_ARRIVAL_TOLERANCE` 需要现场
   根据机械间隙和抖动做最后标定。
