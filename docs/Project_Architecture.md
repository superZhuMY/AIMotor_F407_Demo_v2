# 双机械臂电机控制系统 — 项目架构文档

## 1. 系统概述

基于 STM32F407VET6 的双机械臂电机控制固件。通过 4 路 RS485 总线控制 12 个电机（6 个 AI 电机 + 6 个慕纬度电机），上位机通过 UART 发送文本命令或二进制协议帧（AA 55，详见 API_Protocol_v1.0.md）控制。

## 2. 硬件

| 项目 | 值 |
|------|-----|
| MCU | STM32F407VET6 (Cortex-M4, 168MHz, LQFP100) |
| 开发工具链 | Keil MDK-ARM V5, ARMCLANG V6.24 |
| HAL 库 | STM32Cube FW_F4 V1.28.3 |

### 2.1 引脚分配

| 外设 | 引脚 | 功能 | 波特率 | 所属臂 |
|------|------|------|--------|--------|
| USART1 | PA9(TX)/PA10(RX) | 上位机通信 | 115200-8N1 | — |
| USART2 | PA2(TX)/PA3(RX) | AI 电机总线 #1 | 115200-8N2 | 左 |
| USART3 | PB10(TX)/PB11(RX) | AI 电机总线 #2 | 115200-8N2 | 右 |
| USART6 | PC6(TX)/PC7(RX) | MW 电机总线 #1 | 115200-8N1 | 左 |
| UART5 | PC12(TX)/PD2(RX) | MW 电机总线 #2 | 115200-8N1 | 右 |
| UART4 | PA0(TX)/PA1(RX) | 预留（舵机/串口屏） | — | — |
| PB0 | GPIO_Output | AI 左臂 TX LED | — | — |
| PB2 | GPIO_Output | AI 右臂 TX LED | — | — |
| PE0 | GPIO_Output | MW 左臂 TX LED | — | — |
| PE1 | GPIO_Output | MW 右臂 TX LED | — | — |

### 2.2 电机映射

每条 RS485 总线挂 3 个电机（站号 1/2/3）。左臂和右臂为镜像结构。

**AI 电机（Modbus RTU，脉冲制）：**

| 总线 | 命令前缀 | 轴 | M1 | M2 | M3 |
|------|----------|-----|-----|-----|-----|
| USART2 | B1 (左) | | X(横移) | Z(升降) | Y(伸缩) |
| USART3 | B2 (右) | | X(横移) | Z(升降) | Y(伸缩) |

**MW 电机（私有协议 0x3E，角度制，有耦合）：**

| 总线 | 命令前缀 | ID1(J4) | ID2(J5) | ID3(J6) |
|------|----------|---------|---------|---------|
| USART6 | L (左) | 独立旋转 | Y轴旋转 | Y轴旋转(耦合) |
| UART5 | R (右) | 独立旋转 | Y轴旋转 | Y轴旋转(耦合) |

J5/J6 机械耦合关系：
```
motor5_target = J5_deg × 5/3 × 100
motor6_target = J6_deg × 20/9 × 100 - motor5_target
```

## 3. 软件架构

### 3.1 文件结构

```
App/                      ← 手写应用代码（CubeMX 重新生成不覆盖）
├── Inc/
│   ├── aimotor.h          — AI 电机公共 API（Init/Process/回调/Modbus 命令）
│   ├── aimotor_internal.h — aimotor 家族内部共享符号（控制状态/命令槽/序列类型/自检钩子）
│   ├── host_protocol.h    — 上位机协议层公共 API（Host_ProtocolInit/Host_ReplyBytes）
│   └── mwmotor.h          — MW 电机公共 API
└── Src/
    ├── aimotor.c          — 电机核心：数组/初始化/DMA 回调路由/单电机状态机
    ├── aimotor_modbus.c   — Modbus RTU 编解码与 RS485 发送（CRC16/CDAB/命令帧）
    ├── aimotor_ctrl.c     — 异步控制序列（ENABLE/STOP/DISABLE/上电安全停止/回滚）
    ├── host_protocol.c    — 上位机协议层：控制状态/看门狗/流解析/二进制帧/文本命令
    ├── aimotor_selftest.c — 固件自检（AIMOTOR_SELF_TEST=1 时编译）
    ├── comm_router.c      — HAL UART 回调分发（按外设路由到各模块）
    └── mwmotor.c          — MW 电机私有协议 + 状态机 + J5/J6 耦合解算

Core/                     ← CubeMX 生成区
├── Inc/
│   ├── main.h
│   ├── usart.h        — CubeMX 生成
│   ├── dma.h          — CubeMX 生成
│   └── gpio.h         — CubeMX 生成
└── Src/
    ├── main.c          — 主入口，薄层编排（仅初始化调用与 5ms 调度）
    ├── usart.c         — CubeMX 生成
    ├── dma.c           — CubeMX 生成
    ├── gpio.c          — CubeMX 生成
    └── stm32f4xx_it.c  — 中断服务函数
```

Keil 工程对应分组：`Application/User/App`（App/Src 与 App/Inc 全部文件）与
`Application/User/Core`（main.c 等）；包含路径已含 `../App/Inc`。

### 3.2 模块职责

**main.c** — 主入口（薄层编排）：
- 初始化外设 → Host_ProtocolInit() → Aimotor_Init() → MW_Init()
- 主循环每 5ms 调度 Aimotor_Process()/Aimotor_CommWatchdog() 与 MW_Process()
- UART 回调分发在 comm_router.c；协议缓冲所有权在 host_protocol.c

**aimotor.c + aimotor_modbus.c + aimotor_ctrl.c + host_protocol.c** — AI 电机模块群：
- aimotor.c：电机/总线数据结构、初始化、DMA 回包按从站号路由、单电机运动状态机（STOP→WRITE→TRIGGER，位置由空闲轮询刷新）
- aimotor_modbus.c：Modbus RTU 编解码（06H/10H/03H，CRC16，CDAB 字节序）与 RS485 发送
- aimotor_ctrl.c：异步控制序列（ENABLE/STOP/DISABLE/上电安全停止/失败回滚），每台电机回包确认
- host_protocol.c：控制状态机（DISABLED/ENABLED/STOPPED/FAULT）、250ms 通信看门狗、上位机流解析（环形缓冲拆包/粘包/CRC 重同步 + 文本行提取）、二进制 AA 55 帧与文本 B/ALL 命令解析执行、STATE 上报
- aimotor_selftest.c：固件自检（DRY_RUN 下覆盖生产函数，60 项）
- comm_router.c：HAL UART 回调按外设分发

**mwmotor.c** — MW 电机模块：
- 私有协议实现（帧头 0x3E，和校验）
- J5/J6 耦合解算
- 与 aimotor 相同的轮询 + 状态机架构
- 上位机命令解析（L/R 前缀）

### 3.3 数据流

```
上位机(PC) ──USART1──→ main.c 回调 ─→ Aimotor_HostRxAppend() 环形缓冲
                                              │
                            Aimotor_HostStreamPoll() 主循环解析
                    （拆包/粘包/CRC 重同步 + 文本行提取）
                              │                       │
              二进制帧/B/ALL 文本行              L/R 文本行（转发 g_host_rx_buf）
                              ▼                       ▼
                      Aimotor_Process()         MW_Process()
                              │                       │
                              ▼                       ▼
                        状态机执行                 状态机执行
                        Modbus 命令               0x3E 命令
                              │                       │
                    ┌─────────┤               ┌─────────┤
                    ▼         ▼               ▼         ▼
                 USART2    USART3          USART6    UART5
                 (左AI)    (右AI)          (左MW)    (右MW)
```

### 3.4 核心数据结构

```c
// AI 电机
typedef struct {
    uint8_t  slave_id;          // Modbus 站号 1/2/3
    int32_t  target_position;   // 目标位置(脉冲)
    int32_t  actual_position;   // 实际位置(脉冲)，后台轮询更新
    uint16_t run_speed;         // 速度(rpm)
    Aimotor_Step_t step;        // 状态机步骤
    uint8_t  cmd_pending;       // 上位机命令待处理
    // 私有 RX 缓冲
    uint8_t  rx_buf[16];
    volatile uint8_t rx_ready;
    volatile uint8_t rx_len;
} Aimotor_t;

// 总线
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *led_port;
    uint16_t led_pin;
    uint8_t tx_buf[50];         // 发送缓冲
    uint8_t rx_buf[50];         // 总线级 DMA 缓冲
} Aimotor_Bus_t;

// 全局数组
Aimotor_Bus_t aimotor_buses[2];       // [bus]
Aimotor_t aimotor_motors[2][3];      // [bus][motor]
```

MW 模块结构类似，角度制替换脉冲制，J5/J6 有耦合字段。

## 4. 通信协议

### 4.1 AI 电机 — Modbus RTU

| 寄存器 | 功能 | 功能码 | 值 |
|--------|------|--------|-----|
| H03_03 (0x0303) | 伺服使能 | 06H | 0=断开, 1=导通 |
| H03_05 (0x0305) | 多段位启停 | 10H | 0=停止, 1=触发 |
| H11_12 (0x110C) | 位移量 Int32 + 速度 UInt16 | 10H | CDAB 字节序(低16在前) |
| H0B_07 (0x0B07) | 实际位置 Int32 | 03H | 读2寄存器 |

### 4.2 MW 电机 — 私有协议 0x3E

帧格式：`0x3E + CMD + ID + DataLen + CMD_SUM + [Data + DATA_SUM]`

| 命令 | 功能 | 帧长 |
|------|------|------|
| 0x88 | 电机运行 | 5 |
| 0x80 | 电机关闭 | 5 |
| 0xA4 | 多圈位置控制(带速度限制) | 18 |
| 0x92 | 读取多圈角度 | 5 |

0xA4 数据：int64_t 角度(0.01°/LSB, 小端) + uint32_t 速度(0.01dps/LSB, 小端)

### 4.3 上位机文本命令

通过 USART1 (115200-8N1) 发送，以 `\r\n` 结尾。

| 命令 | 功能 | 回复 |
|------|------|------|
| `B1 M1 P10000 S300` | 左臂 X 轴运动 | `OK B1 M1 P10000 S300` |
| `B2 M3 P-5000 S500` | 右臂 Y 轴运动 | `OK B2 M3 P-5000 S500` |
| `B1 M1 Q` | 查询位置(缓存) | `POS B1 M1 12345` |
| `B1 M1 EN` | 单轴使能已禁用 | `ERR: single-axis EN disabled, use ALL ENABLE` |
| `B1 STOP` | 全局停止双臂 12 轴 | `OK STOP: ACCEPTED` |
| `L J5 30 J6 45 S180` | 左臂关节角度 | `OK L J5=30 J6=45 S=180` |
| `L Q` | 查询关节角度 | `POS L J5=30 J6=45` |
| `ALL STOP` | 全部急停 | `OK STOP: ACCEPTED` |

### 4.4 AI 驱动器手册要点（AIMotor 低压系列使用手册，v1.4 核实）

**控制方案（手册 §5.2 通讯控制位置运行）**：驱动器预设置 H03_04=28（DI2 关联
InFun28 多段位使能 PosInSen）后，H03_05 即该虚拟使能端子的控制位——写 1 多段位
开始运行（按 H11_12 位移 / H11_14 速度 / H11_15 加减速），写 0 停止。多段位使能
**电平有效、运行结束自动停止，但"再次运行需要重新导通多段位使能"**——因此每次
运动前必须先写 0x0305=0 再触发，现有 STOP→WRITE→TRIGGER 序列与手册一致，STOP
不可省。

**波特率上限**：H0C_02 设置值 0~6，最高 115200——序列往返削减是唯一软件提速杠杆。

**寄存器级发现（按需启用，当前均未实现）**：

| 能力 | 寄存器入口 | 说明 |
|------|-----------|------|
| 一帧写位移+速度 | 10H 写 0x110C 起 3 寄存器 | H11_12(Int32)+H11_14(UInt16) 连续（手册 §5.1-5 原例）；当前速度固定用驱动器内预置值 |
| 一次读位置+负载率+偏差 | 03H 读 0x0B07 起 15 寄存器 | H0B_07 位置 / H0B_12 平均负载率 / H0B_15 偏差连续可读；开启到位判断时可用一帧取代 QUERY+QUERY_ERROR 两步 |
| 通讯触发回零/置零 | H05_30=4/5（原点搜索/电气回零）、=6（当前位置清零，H0B_07 置 0） | 驱动器内置回零状态机（H05_31 共 16 种模式），无需固件自造；命令执行完 H05_30 自动置 0，勿循环发送 |
| 紧急停机 | H0D_05 写 1 | 立即停止并保持位置锁定；写 0 解除（比 0x0305=0 更强，急停语义） |
| 故障诊断 | H0B_33（选择前 n 次）+ H0B_34（故障码） | 未来 STATE 帧扩展诊断信息的来源；通信写参数是否存 EEPROM 由 H0C_13 控制 |
| Modbus 异常帧 | 功能码 83H/86H/90H + 错误号 | 当前实现把异常帧当作无效回包走重试，未来可解析错误号区分"参数错"与"忙" |

## 5. 状态机设计

### 5.1 空闲轮询（后台持续运行）

```
IDLE → QUERY → WAIT_RX → (收到回包) → ProcessRx → IDLE
                                  ↓
                          更新 actual_position
```

每电机通过 round-robin 调度（5ms tick，每 tick 每总线处理一个电机），约每 15ms 轮到一次查询。回包通过 DMA 回调按从站号路由到对应电机的私有缓冲，避免同总线电机互相覆盖。

### 5.2 运动序列（上位机命令触发）

```
IDLE → (cmd_pending=1) → STOP → WRITE → TRIGGER → IDLE
（序列内每步带回包校验与 10 次超时重试；v1.4 起 QUERY 移交空闲轮询，
  位置反馈经 IDLE→QUERY→WAIT_RX→IDLE 持续刷新）
```

每个步骤在独立 tick 中执行（5ms tick；同一总线存在未完成请求时该总线只推进该事务，保证 RS485 半双工一次一帧）。

### 5.3 DMA RX 回包处理

```
HAL_UARTEx_RxEventCallback (comm_router.c)
  ├── USART1? → g_host_dma_buf → Aimotor_HostRxAppend() 追加入环形缓冲并重启 DMA
  │              （主循环 Aimotor_HostStreamPoll() 完成拆包/CRC/文本行提取）
  ├── AI 总线? → Aimotor_RxCallback → 按 slave_id 路由到电机私有缓冲
  └── MW 总线? → MW_RxCallback → 按 ID 路由到电机私有缓冲
```

### 5.4 命令分发（避免消费竞争）

```
Aimotor_HostStreamPoll()（host_protocol.c，主循环）:
  二进制帧  → CRC 校验后执行（ACK/STATE 应答，有效帧刷新看门狗）
  B/A 文本行 → 写入 g_host_cmd_buf（g_host_cmd_ready=1）
  L/R 文本行 → 转发写入 g_host_rx_buf（g_host_rx_ready=1）

Aimotor_Process():
  if g_host_cmd_ready && 首字符∈{B,A} → 消费，解析 B/ALL 命令

MW_Process():
  if g_host_rx_ready && 首字符∈{L,R} → 消费，解析 L/R 命令
```

两个模块各自只消费自己的命令前缀，互不干扰。

## 6. 关键设计决策

| 决策 | 理由 |
|------|------|
| **每电机私有 RX 缓冲** | 3 个电机共享一条 RS485 总线，只有一个 DMA 缓冲。回包到达时按从站号拷贝到对应电机私有缓冲，避免后续回包覆盖 |
| **后台持续轮询** | MoveIt 需要 20-50Hz 位置反馈。空闲时自动持续查询，缓存始终最新，Q 命令直接返回缓存（零延迟） |
| **文本命令协议** | 优于二进制协议：调试直观、跨语言通用、20 字节在 115200 下 ~1.8ms 可忽略 |
| **06H vs 10H 功能码** | 伺服使能(0x0303)用 06H 可用，但启停(0x0305)和位置(0x110C)必须用 10H，与 CPP 参考代码一致 |
| **模块分离** | AI 电机(Modbus)和 MW 电机(私有协议)各自独立模块，数据结构、状态机、协议函数完全隔离 |
| **ROUND_ROBIN 调度** | 每条总线每次 tick 只处理一个电机，避免半双工 RS485 冲突。3 tick 轮完一圈 |
| **五模块拆分** | aimotor.c 按职责拆为电机核心/Modbus 编解码/控制序列/上位机协议/自检五个文件，公共 API 不变；跨文件内部符号收敛在 aimotor_internal.h |
| **非阻塞回复通道** | 上位机回复（ACK/STATE/文本）统一走 Host_ReplyBytes 中断发送，5ms 调度循环不再被 70 字节 STATE 帧 ~6ms 的阻塞发送拖住；发送前先拷入持久静态缓冲 `g_host_tx_async_buf[80]`（`HAL_UART_Transmit_IT` 不复制数据，直传栈数组会在异步发送期间失效），等待上一帧超时则丢弃本帧而不覆盖在发缓冲（v1.4） |
| **环形缓冲流解析** | USART1 的 ISR 只做字节追加，拆包/粘包/CRC 重同步全部在主循环完成；解析有迭代上限，绝不阻塞 |

## 7. 待完善项

- [x] MW 电机 J4 独立关节角度命令（L J4 HOME / L J4 <角度> 已支持）
- [x] 上位机连接超时自动急停（250ms 通信看门狗，v1.3）
- [x] Phase 2：12 轴（2 臂 × 6 关节）全部接入并经 STATE 帧上报
- [ ] 开启到位判断：AIMOTOR_ENABLE_ARRIVAL_CHECK / MW_ENABLE_ARRIVAL_CHECK 由 0 改 1 并现场标定容差
- [ ] 串口屏显示模块（UART4, PA0/PA1；与遥控接收机用途二选一）
- [ ] 运动到达自动上报 / 周期 STATE 主动流（协议 VERSION 升版）
- [ ] 错误状态上报（电压、温度、堵转等，经驱动器状态寄存器读入 STATE 帧）
- [ ] 上位机正式 ROS 节点：消费 STATE 关节侧 µm/µrad，删除旧节点的 LINEAR_FACTORS 二次换算（参考 tools/host_reference/）
- [x] 提高总线波特率——**不可行**：驱动器波特率上限即 115200（手册 H0C_02 设置值 0~6）；序列往返削减是唯一软件杠杆（v1.4 已砍 QUERY）
- [ ] **USART2/USART3 停止位待实机确认**：`.ioc` 写 `USART2.StopBits=STOPBITS_2`（USART3 无该键，默认 1），而 `Core/Src/usart.c` 五个 UART 全为 `UART_STOPBITS_1`，文档（§2.1 本表、README_J456_DRIVER）却称 AI 总线为 8N2 → 当前固件实际跑 **8N1**。需示波器/逻辑分析仪确认 AI 总线真实帧格式，再对齐 `.ioc`、`usart.c`、文档三者；否则下次用 CubeMX 从该 `.ioc` 重新生成会静默把 USART1/USART2 改成 2 停止位（USART1 按文档应为 8N1，会改坏上位机链路）

## 8. 时序与性能预算

线上参数：115200 8N2 ≈ 95.5µs/字节（11 bit）；Modbus RTU 帧间静默 ≈ 0.3ms。

| 路径 | 组成 | 实测量级 |
|------|------|---------|
| AI 单次总线事务 | 发 8~13B + 驱动器响应 + 收 8~9B | 3~6ms（大头是驱动器响应时间） |
| AI 单次运动序列 | STOP→WRITE→TRIGGER 共 3 次往返（v1.4 砍 QUERY） | ~12~19ms，与同总线 2 电机轮转共享 |
| MW 耦合事务 | 写 M5→写 M6→读 M5→读 M6 共 8 次往返 | 全系统最慢路径 |
| 空闲轮询 | AI 每电机 ~15ms；MW J4 ~100ms、J5/J6 随耦合事务 | MoveIt 反馈短板在 MW 侧 |
| ENABLE/STOP 序列 | 12 轴逐台回包确认，每 tick 推进一步 | ~0.5~2s（一次性操作） |
| 5ms tick 上限 | 每 tick 每总线 1 个事务 | 理论 200 事务/总线/秒 |

**瓶颈结论**：MCU（168MHz，RX 全 DMA）不是瓶颈；瓶颈在半双工应答等待 × 往返次数。
提升杠杆排序：驱动器波特率 ↑ > 砍运动序列往返（QUERY 交给空闲轮询）> 控制序列按总线并行 > TX 中断化。

## 9. 构建、烧录与验收

### 9.1 编译配置矩阵

| 配置 | AIMOTOR_DRY_RUN | AIMOTOR_SELF_TEST | 用途 |
|------|-----------------|-------------------|------|
| 发布 | 0 | 0 | 实机运行 |
| 干跑 | 1 | 0 | 无硬件验证总线命令序列（mock 日志） |
| 自检 | 1 | 1 | 上电跑 ~60 项自检（宏在 aimotor.h 带 #ifndef，可由构建配置覆盖） |

### 9.2 步骤

1. Keil 打开 MDK-ARM/AIMotor_F407_Demo.uvprojx，Rebuild（应用代码在 Application/User/App 组）
2. 自检配置烧录 → USART1 (115200) 查看逐项 PASS/FAIL，预期 0 failed
3. 换发布配置烧录 → 上位机 USART1 走 HELLO → ENABLE → TARGET → STATE 二进制链路验收
4. 主机端回归：python tools/protocol_test.py（60 项，需真 Python）

### 9.3 本地无编译器时的快速自检（详见 tools/dev/README.md）

```
node tools/dev/balance_check.js      # 括号平衡
node tools/dev/syntax_check.js       # tree-sitter 语法检查（对比 HEAD 基线，只报新错误）
doxygen docs/Doxyfile                # 代码 API 文档（docs/api_html/index.html）
```

注意：以上只覆盖语法结构层，查不出隐式声明/重声明等语义错误——**最终以 Keil 编译为准**。

## 10. 文档体系

| 文档 | 内容 | 维护时机 |
|------|------|---------|
| 本文件（Project_Architecture.md） | 架构/数据流/状态机/时序/验收 | 每次结构调整 |
| API_Protocol_v1.0.md | 上位机二进制协议线上字节定义 | 协议变更（升 VERSION） |
| docs/api_html/（Doxygen 生成） | 代码级 API：全部函数/结构体/宏 | 注释在头文件，随改随生成 |
| README_J456_DRIVER.md | J4/J5/J6 驱动说明 | 驱动层变更 |
| docs/MODIFICATION_REPORT_v1.3.md | 变更记录（v1.3 安全返修） | 每次返修追加 |
| tools/dev/README.md | Windows 环境与本地自检工具 | 环境变化 |

## 11. 已知问题与 FAQ

- **链接错误 L6031U（找不到 .sct）**：散布文件位于构建输出目录但作为固定链接输入，已入库（.gitignore 例外）；新克隆后直接 Rebuild 即可
- **Keil 控制台中文注释乱码**：GBK 控制台显示 UTF-8 文件，仅显示问题，文件无损
- **git 提示 LF→CRLF / git log 中文乱码**：autocrlf 自动转换与终端编码显示问题，仓库数据无损（UTF-8）
- **python 命令无输出退出码 49**：WindowsApps 占位 stub，需安装真 Python（winget install Python.Python.3.12）
