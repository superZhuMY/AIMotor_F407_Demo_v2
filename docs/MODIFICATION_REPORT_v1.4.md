# STM32 双臂下位机发送路径修复说明（v1.4）

本次为**低风险小修改**，只修发送路径上的两个缺陷，不改控制架构。共 3 个应用
文件、27 行新增、17 行删除。

## 1. 删除 AI/MW 发送层重复的 `UART_FLAG_TC` 等待

涉及 `App/Src/aimotor_modbus.c` 的 `Aimotor_BusSendBytes()` 与
`App/Src/mwmotor.c` 的 `MW_SendBytes()`。

两处都在 `HAL_UART_Transmit()` 返回后又用 `__HAL_UART_GET_FLAG(huart,
UART_FLAG_TC)` 自旋等一个超时窗口。**该等待是冗余的**：本仓库 HAL 源码
`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c` 的 `HAL_UART_Transmit()`
在发完最后一个字节后已经执行
`UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TC, RESET, tickstart, Timeout)`，
函数返回即表示停止位已完全送出。删除后：

- 每条总线事务少一次无意义的标志轮询与 `HAL_GetTick()` 调用；
- 超时语义不变——TC 等待的超时值与 `HAL_UART_Transmit` 传入的超时值是同一个
  （`AIMOTOR_DMA_TIMEOUT` / `MW_TIMEOUT_MS`），删除循环不会缩短任何真实等待；
- 仍使用**阻塞** `HAL_UART_Transmit`，未引入 TX DMA、TX 中断或发送队列。

LED 置位/复位、发送失败提前返回、`g_dry_run_*_tx_count++`、`AIMOTOR_DRY_RUN`
分支均未改动。

## 2. 修复 USART1 异步发送的缓冲区生命周期问题

涉及 `App/Src/host_protocol.c` 的 `Host_ReplyBytes()`。

`HAL_UART_Transmit_IT()` 不复制数据，只保存源缓冲区地址。而调用者中有两个使用
**局部栈数组**：`BinarySendAck()` 的 `out[14]` 与 `Aimotor_SendState()` 的
`out[70]`。原实现把栈地址直接交给中断发送，函数返回后栈帧失效，异步发送期间
数据可能被后续调用覆盖，表现为偶发的 ACK/STATE 帧内容错乱。

修复：新增持久静态缓冲，所有 IT 发送只从该缓冲发出。

```c
#define HOST_TX_BUF_SIZE 80U                       /* 覆盖最大 STATE 帧 70B */
static uint8_t g_host_tx_async_buf[HOST_TX_BUF_SIZE];
```

`Host_ReplyBytes()` 的语义变化：

| 环节 | 修改前 | 修改后 |
|------|--------|--------|
| 长度校验 | 仅拒绝 `NULL` / `0` | 增加 `len > HOST_TX_BUF_SIZE` 拒绝 |
| 等待上一帧 | 超时 `break`，继续往下发 | 超时 `return`，**丢弃本帧而不覆盖在发的缓冲** |
| 数据源 | 直接用调用方指针 | `memcpy` 到 `g_host_tx_async_buf` 后再发 |
| IT 启动失败回退 | 阻塞发送调用方指针，超时 100ms | 阻塞发送静态缓冲，超时 `HOST_TX_WAIT_MS` |

"等待超时则丢弃"不是功能缩水：旧代码超时后 `gState` 仍为 `BUSY`，
`HAL_UART_Transmit_IT()` 与随后的阻塞 `HAL_UART_Transmit()` 都会立即返回
`HAL_BUSY`，那一帧本来也无法发出。新写法只是把"丢帧"变成确定行为，并消除了竞态。

安全性核对：`Host_ReplyBytes()` 仅由主循环路径调用（`Aimotor_Process()` 与
`MW_Process()`，见 `Core/Src/main.c`），中断上下文不经过它，静态缓冲无重入问题；
`comm_router.c` 只有 RX 回调，无 TX 路径。工程未自定义 `HAL_UART_TxCpltCallback`，
HAL 默认实现会把 `gState` 正常恢复为 `READY`，IT 通道原有工作方式不变。

ACK、STATE、文本回复仍全部统一走 `Host_ReplyBytes()`，二进制协议格式未改。
DRY_RUN 下 `AIMOTOR_SELF_TEST` 打开时两个函数都在调用 `Host_ReplyBytes()` 前
写入 `g_test_tx_log` 并提前返回，自检观测行为不变。

## 3. 明确未改动的部分

AI/MW 的 TX DMA 与 TX 中断、TX 消息队列、FreeRTOS、新 UART TX 状态机、5ms 主
循环、控制序列状态机、Modbus 状态机、MW J5/J6 耦合状态机、250ms 通信看门狗、
STOP/ENABLE/DISABLE 逻辑、TARGET/STATE 协议结构、到位判断逻辑——本次均未触碰。

## 4. 已执行验证

| 检查 | 结果 |
|------|------|
| `node tools/dev/balance_check.js` | 通过（退出码 0） |
| `node tools/dev/syntax_check.js` | 通过，"无相对 HEAD 的新错误" |
| 协议回归（60 项） | `60 passed, 0 failed`（退出码 0） |

协议回归使用 `tools/dev/protocol_test.js` —— 本机无可用 Python
（`python`/`python3` 均为 WindowsApps 占位存根，退出码 49；`py` 不存在），
该脚本为 `tools/protocol_test.py` 的 Node 等价移植，断言与计数值 1:1 对齐，
Python 语义点显式对齐（`//` → `pdiv`，`round()` 半值向偶数 → `pround`）。
**权威行为仍以 `protocol_test.py` 为准**，装好真 Python 后建议复跑一次。

本机无 Keil ARMClang，因此未生成新的 `.hex/.axf`，也未上板运行 C 自检。
上述两个脚本只做语法结构层检查，查不出语义/链接期问题，**必须以 Keil 编译为
最终门**。

## 5. 待实机确认项：USART2/USART3 停止位

仓库内部三处配置互相矛盾，**本次未修改任何停止位**：

| 来源 | USART1 | USART2（左 AI） | USART3（右 AI） |
|------|--------|-----------------|-----------------|
| `AIMotor_F407_Demo.ioc` | `STOPBITS_2` | `STOPBITS_2` | 无该键（默认 = 1） |
| `Core/Src/usart.c`（运行期实际生效） | `UART_STOPBITS_1` | `UART_STOPBITS_1` | `UART_STOPBITS_1` |
| `Project_Architecture.md` / `README_J456_DRIVER.md` | 8N1 | **8N2** | **8N2** |

当前固件在两根 AI 总线上**实际运行的是 8N1**，与文档声称的 8N2 不一致。

风险提示：若后续用 CubeMX 从这份 `.ioc` 重新生成代码，USART1 与 USART2 会被
静默改为 2 个停止位；按文档 USART1 应为 8N1，这会直接改坏上位机链路。

**结论：需要实机参数确认。** 建议用示波器或逻辑分析仪测量 AI 总线实际帧格式，
确认后一次性对齐 `.ioc`、`usart.c` 与文档三者（或删掉 `.ioc` 中这两处手工
`StopBits` 项让默认值生效）。

## 6. 建议烧录前步骤

1. 以 `AIMOTOR_DRY_RUN=1`、`AIMOTOR_SELF_TEST=1` 构建并烧录，确认 USART1 打印
   `0 failed`；
2. 恢复发布配置 `AIMOTOR_DRY_RUN=0`、`AIMOTOR_SELF_TEST=0` 全量重编译；
3. 电机断供电抓包，确认 ACK 14 字节与 STATE 70 字节在连续高频请求下内容稳定
   （本次缓冲区修复的验收点）、且上位机链路无丢帧；
4. 通过上述步骤后再进行带动力低速、小位移测试。
