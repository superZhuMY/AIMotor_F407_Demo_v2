/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : aimotor.h
  * @brief          : AI Motor multi-bus RS485 Modbus RTU control module
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __AIMOTOR_H__
#define __AIMOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* ── 系统常量 ─────────────────────────────────────────────────── */
#define AIMOTOR_BUS_COUNT      2      // Phase 1: 2 路 RS485 总线
#define AIMOTOR_MOTORS_PER     3      // 每路 3 个电机
#define AIMOTOR_TOTAL          6      // 共 6 个电机 (Phase 2 扩展到 12)

#define AIMOTOR_BUF_SIZE       50     // TX/RX 缓冲区大小
#define AIMOTOR_POSITION_COMPLETE_THRESHOLD_CMD 92U
#define AIMOTOR_ENCODER_PER_COMMAND_NUM        131072LL
#define AIMOTOR_ENCODER_PER_COMMAND_DEN        1000LL
#define AIMOTOR_POS_ERROR_TOLERANCE_ENCODER \
    ((int32_t)((AIMOTOR_POSITION_COMPLETE_THRESHOLD_CMD * \
                AIMOTOR_ENCODER_PER_COMMAND_NUM + \
                AIMOTOR_ENCODER_PER_COMMAND_DEN - 1LL) / \
               AIMOTOR_ENCODER_PER_COMMAND_DEN))
#define AIMOTOR_AWAY_PULSE     200000 // 远端目标脉冲数
#define AIMOTOR_HOME_PULSE     0      // 回零目标脉冲数
#define AIMOTOR_TICK_MS        5      // 状态机调度周期(ms)
#define AIMOTOR_RETRY_MAX      10     // 等待回包超时重试次数
#define AIMOTOR_DMA_TIMEOUT    20     // DMA 超时(ms)
#define AIMOTOR_RESPONSE_TIMEOUT_MS  30
#define AIMOTOR_MOVE_TIMEOUT_MS      5000
#define AIMOTOR_ARRIVE_SAMPLES       3
/* 测试阶段暂不启用到位判断；保留完整代码，实机通信稳定后改为 1。 */
#define AIMOTOR_ENABLE_ARRIVAL_CHECK 0

#define AIMOTOR_POSITION_MIN   (-10000000L)
#define AIMOTOR_POSITION_MAX   ( 10000000L)

/* ── 上位机二进制协议（固定帧格式，与 docs 协议文档保持一致） ────── */
#define AIMOTOR_BINARY_SOF0        0xAA
#define AIMOTOR_BINARY_SOF1        0x55
#define AIMOTOR_BINARY_VERSION     0x01
/* AA 55 | VERSION(1) | CMD(1) | SEQ(2) | LEN(2) | PAYLOAD(LEN) | CRC16(2) */
#define AIMOTOR_BINARY_HEADER_SIZE 8U     /* 帧头固定 8 字节 */
#define AIMOTOR_BINARY_TOTAL(n)    (AIMOTOR_BINARY_HEADER_SIZE + (n) + 2U)

/* 命令码 */
#define AIMOTOR_BINARY_CMD_HELLO      0x01
#define AIMOTOR_BINARY_CMD_ENABLE     0x02
#define AIMOTOR_BINARY_CMD_STOP       0x03
#define AIMOTOR_BINARY_CMD_GET_STATE  0x04
#define AIMOTOR_BINARY_CMD_DISABLE    0x05
#define AIMOTOR_BINARY_CMD_TARGET     0x10
/* 响应命令码 */
#define AIMOTOR_BINARY_RESP_ACK       0x80
#define AIMOTOR_BINARY_RESP_STATE     0x81

/* ACK 结果码（ACK payload: SEQ(2)+CMD(1)+result(1)，LEN=4） */
#define AIMOTOR_ACK_OK              0x00  /* 已受理（解析+状态检查通过） */
#define AIMOTOR_ACK_CRC             0x01  /* CRC 校验失败 */
#define AIMOTOR_ACK_FORMAT          0x02  /* 帧格式错误/长度与 LEN 不符 */
#define AIMOTOR_ACK_UNKNOWN_CMD     0x03  /* 未知 CMD */
#define AIMOTOR_ACK_BAD_LEN         0x04  /* 该 CMD 的 payload 长度错误 */
#define AIMOTOR_ACK_OUT_OF_RANGE    0x05  /* 关节参数越界 */
#define AIMOTOR_ACK_STATE_DENIED    0x06  /* 当前控制状态不允许该命令 */
#define AIMOTOR_ACK_BAD_ARM         0x07  /* 无效 arm */
#define AIMOTOR_ACK_COMM_TIMEOUT    0x08  /* 通信超时，需重新 ENABLE */

/* 流解析（拆包/粘包/CRC 重同步） */
#define PROTOCOL_MAX_PAYLOAD        60U   /* 最大 payload（STATE=60） */
#define PROTOCOL_RX_BUFFER_SIZE     256U
#define HOST_TEXT_LINE_TIMEOUT_MS   50U   /* 无 \n 的文本行冲刷超时 */
#define HOST_COMM_WATCHDOG_MS       250U  /* 主机通信看门狗 */

/**
 * @brief 控制状态机：DISABLED 上电默认（拒 TARGET）/ ENABLED 允许运动 /
 *        STOPPED 停止后需显式 ENABLE / FAULT 故障锁定
 */
typedef enum {
    CONTROL_DISABLED = 0,   /* 上电默认；拒绝 TARGET */
    CONTROL_ENABLED,        /* 允许 TARGET */
    CONTROL_STOPPED,        /* 停止后；需显式 ENABLE 才能运动 */
    CONTROL_FAULT
} ControlState_t;

/** @brief 控制级故障标志（组合位，经 STATE 帧 fault 字节上报） */
#define CONTROL_FAULT_NONE          0x00U
#define CONTROL_FAULT_COMM_TIMEOUT  0x01U
#define CONTROL_FAULT_RX_OVERFLOW   0x02U
#define CONTROL_FAULT_ENABLE_FAILED 0x04U      /* ENABLE 必要电机失败 */
#define CONTROL_FAULT_UNCONFIRMED_STOP 0x08U   /* STOP/DISABLE 存在未确认停止的电机 */
#define CONTROL_FAULT_INTERNAL      0x10U      /* 内部状态错误 */

/* 新 ACK 结果码（详见协议文档 §6.4） */
#define AIMOTOR_ACK_CTRL_BUSY       0x09   /* 控制序列进行中，命令被拒绝 */
#define AIMOTOR_ACK_CTRL_FAILED     0x0A   /* 控制命令部分/全部电机失败 */

/** @brief 异步控制序列种类（步骤表见 aimotor_internal.h） */
#define CTRL_RETRY_MAX              3      /* 控制序列单电机重试上限 */
typedef enum {
    CTRL_SEQ_NONE = 0,
    CTRL_SEQ_STARTUP_SAFE_STOP,   /* 上电：全轴安全停止/禁用 */
    CTRL_SEQ_ENABLE,              /* 全轴 ServoOn + 0x88 */
    CTRL_SEQ_STOP,                /* 全轴停止（最高优先级，可抢占） */
    CTRL_SEQ_DISABLE,             /* 全轴停止 + Servo Off + 0x80 */
    CTRL_SEQ_ROLLBACK             /* ENABLE 失败：全 12 轴安全回滚 */
} CtrlSeqKind_t;

/** @brief 二进制帧分发结果（决定是否刷新通信看门狗） */
typedef enum {
    FRAME_REJECTED = 0,               /* 未接受：不刷新 */
    FRAME_ACCEPTED_NO_WATCHDOG,       /* 已接受但不刷新（HELLO 固定不刷新） */
    FRAME_ACCEPTED_REFRESH_WATCHDOG   /* 已接受并刷新 */
} FrameDispatchResult_t;

/* 12 轴轴序位图（side*6 + joint，bit0..5=L_J1..L_J6，bit6..11=R_J1..R_J6） */
#define AIMOTOR_AXIS_BIT(side, joint) ((uint16_t)(1U << ((side) * 6U + (joint))))

/** @brief 单电机运动/轮询步骤：运动序列 STOP→WRITE→TRIGGER 后回 IDLE（v1.4）；
 *         QUERY/WAIT_QUERY 供空闲轮询位置刷新与到位判断使用 */
typedef enum {
    MOTOR_STEP_IDLE    = 0,   // 空闲（无命令可执行）
    MOTOR_STEP_STOP    = 1,   // 发送停止 (10H -> 0x0305=0)
    MOTOR_STEP_WAIT_STOP = 2,
    MOTOR_STEP_WRITE   = 3,   // 写位置 (10H -> 0x110C)
    MOTOR_STEP_WAIT_WRITE = 4,
    MOTOR_STEP_TRIGGER = 5,   // 触发运行 (10H -> 0x0305=1)
    MOTOR_STEP_WAIT_TRIGGER = 6,
    MOTOR_STEP_QUERY   = 7,   // 查询位置 (03H -> 0x0B07)
    MOTOR_STEP_WAIT_QUERY = 8,
    MOTOR_STEP_QUERY_ERROR = 9, // 查询位置偏差 (03H -> 0x0B15)
    MOTOR_STEP_WAIT_ERROR = 10,
    MOTOR_STEP_ARRIVED = 11,
    MOTOR_STEP_FAULT = 12,
} Aimotor_Step_t;

/** @brief 上位机文本命令类型（B/ALL 前缀） */
typedef enum {
    AIMOTOR_CMD_NONE = 0,
    AIMOTOR_CMD_MOVE,
    AIMOTOR_CMD_QUERY,
    AIMOTOR_CMD_ENABLE,
    AIMOTOR_CMD_HOME,
    AIMOTOR_CMD_STOP_BUS,
    AIMOTOR_CMD_STOP_ALL,
    AIMOTOR_CMD_TEST,
    AIMOTOR_CMD_GO,
    AIMOTOR_CMD_ENABLE_ALL,
    AIMOTOR_CMD_DISABLE_ALL
} Aimotor_CmdType_t;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/**
 * @brief 单个 AI 电机的运行状态与私有回包缓冲
 *
 * rx_buf/rx_ready 由 DMA 回调按从站号路由填充，避免同总线多电机回包互相覆盖；
 * enabled 只反映"回包确认后"的实际使能状态。
 */
typedef struct {
    uint8_t  slave_id;          // 电机站号 (1/2/3)
    int32_t  target_position;   // 目标位置(脉冲)
    int32_t  actual_position;   // 实际位置(脉冲)
    int32_t  position_error;    // H0B_15编码器位置偏差
    Aimotor_Step_t step;        // 状态机步骤
    uint8_t  retry_count;       // WAIT_RX 超时计数
    uint8_t  cmd_pending;       // 上位机命令待处理标志
    uint8_t  enabled;           // 伺服使能状态
    uint8_t  arrived;           // 连续满足到位条件
    uint8_t  arrive_count;      // 到位去抖计数
    uint8_t  fault;             // 通信或驱动故障
    uint32_t last_tx_tick;
    uint32_t move_start_tick;
    uint8_t  rx_buf[16];        // 私有回包缓冲（避免同总线电机覆盖）
    volatile uint8_t rx_ready;  // 该电机收到回包
    volatile uint8_t rx_len;    // 回包长度
    uint8_t  actual_valid;      // actual_position 来自最近成功回读
} Aimotor_t;

/** @brief 单路 RS485 总线的收发资源：UART 句柄、TX LED、DMA 缓冲 */
typedef struct {
    UART_HandleTypeDef *huart;  // USART 句柄
    GPIO_TypeDef       *led_port; // TX LED 端口
    uint16_t            led_pin;  // TX LED 引脚
    uint8_t  tx_buf[AIMOTOR_BUF_SIZE];  // 发送缓冲区
    uint8_t  rx_buf[AIMOTOR_BUF_SIZE];  // DMA 接收缓冲区
    volatile uint8_t  rx_ready;         // 接收完成标志
    volatile uint16_t rx_len;           // 接收数据长度
    uint8_t  current_motor;    // 当前轮转电机索引 (0/1/2)
} Aimotor_Bus_t;

/* ── 全局变量声明（定义在 aimotor.c） ─────────────────────────── */
extern Aimotor_Bus_t aimotor_buses[AIMOTOR_BUS_COUNT];
extern Aimotor_t     aimotor_motors[AIMOTOR_BUS_COUNT][AIMOTOR_MOTORS_PER];

/** @brief 上位机文本命令的解析结果 */
typedef struct {
    Aimotor_CmdType_t type;
    uint8_t  bus_idx;      // 总线号 0~3
    uint8_t  motor_idx;    // 电机号 0~2
    int32_t  position;     // 目标位置
    uint8_t  valid;        // 命令是否有效
} Aimotor_Cmd_t;

/* ── API 函数声明 ───────────────────────────────────────────── */
/**
 * @brief  初始化 AI 电机系统：配置总线 GPIO/LED、清零电机参数、启动各总线
 *         DMA 空闲接收，并启动上电安全停止序列（12 轴 Servo Off / 0x80）
 * @note   在 Host_ProtocolInit() 之后、主循环之前调用一次
 */
void Aimotor_Init(void);

/**
 * @brief  AI 电机 5ms tick 处理：控制序列推进、B/ALL 文本命令消费、
 *         单电机状态机轮转（每 tick 每总线至多一个事务，半双工一次一帧）
 * @note   主循环按 AIMOTOR_TICK_MS 周期调用；内含控制序列独占总线的暂停逻辑
 */
void Aimotor_Process(void);

/**
 * @brief  AI 总线 DMA 空闲回包路由（ISR 上下文调用）
 *         按从站号把回包拷入对应电机私有缓冲，并重启本总线 DMA 接收
 * @param  huart 触发回调的 UART 句柄（USART2/USART3）
 * @param  Size  本次 DMA 收到的字节数
 */
void Aimotor_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief  AI 总线接收故障恢复：中止接收、清错误标志、重启 DMA 空闲接收
 * @note   由 comm_router.c 的 UART 错误回调调用
 */
void Aimotor_RecoverRx(UART_HandleTypeDef *huart);

/**
 * @brief  上位机字节流解析（主循环调用，实现在 host_protocol.c）
 *         拆包/粘包、CRC 校验与重同步、二进制帧分发、文本行提取与提交；
 *         解析有迭代上限，绝不阻塞主循环
 */
void Aimotor_HostStreamPoll(void);

/**
 * @brief  上位机 DMA 接收字节追加（ISR 上下文调用，仅拷贝入环形缓冲）
 * @param  data 本次收到的字节
 * @param  len  字节数；环形缓冲满时置 RX_OVERFLOW 故障位，不覆盖未解析数据
 */
void Aimotor_HostRxAppend(const uint8_t *data, uint16_t len);

/**
 * @brief  主机通信看门狗：ENABLED 下 250ms 无有效命令 → 清全部待执行目标、
 *         置 STOPPED 并启动全局停止序列；需重新 ENABLE 才能恢复运动
 */
void Aimotor_CommWatchdog(void);

/** @brief 刷新看门狗时间戳（仅被接受且需刷新的二进制命令/序列成功完成时） */
void Aimotor_RefreshHostWatchdog(void);

/** @brief 查询当前控制状态（DISABLED/ENABLED/STOPPED/FAULT） */
ControlState_t Aimotor_GetControlState(void);

/* ── DRY_RUN 干跑模式（mock 记录待发送命令，真实发送函数调用次数恒为 0） ── */
#define AIMOTOR_DRY_RUN_LOG_MAX    8
#define AIMOTOR_DRY_RUN_FRAME_MAX  16
typedef struct {
    uint8_t  data[AIMOTOR_DRY_RUN_FRAME_MAX];
    uint8_t  len;
    uint8_t  is_mw;      /* 0=AI 总线，1=MW 总线 */
} DryRunLogEntry_t;
extern volatile uint32_t g_dry_run_ai_tx_count;   /* 真实 AI 发送计数（DRY_RUN 下恒为 0） */
extern volatile uint32_t g_dry_run_mw_tx_count;   /* 真实 MW 发送计数（DRY_RUN 下恒为 0） */
extern volatile DryRunLogEntry_t g_dry_run_log[AIMOTOR_DRY_RUN_LOG_MAX];
extern volatile uint8_t  g_dry_run_log_index;     /* 环形写指针 */
/**
 * @brief  记录一条"将发送"的命令帧到 DRY_RUN mock 日志（环形覆盖）
 * @param  is_mw 0=AI 总线帧，1=MW 总线帧
 * @param  data  待记录的命令帧
 * @param  len   帧长（超长截断到 AIMOTOR_DRY_RUN_FRAME_MAX）
 * @note   仅 AIMOTOR_DRY_RUN=1 时由发送层调用；真实发送计数恒为 0
 */
void Aimotor_DryRunLog(uint8_t is_mw, const uint8_t *data, uint16_t len);

/* 安全控制接口（异步控制序列，非阻塞；ACK 在序列完成后回发） */

/**
 * @brief  控制序列（ENABLE/STOP/DISABLE/回滚）是否正在进行
 * @retval 1 进行中：运动命令与新序列被拒绝/排队
 */
uint8_t Aimotor_CtrlSeqActive(void);

/**
 * @brief  发起异步全局安全停止（12 轴 0x0305=0 / 0x80，逐台回包确认）
 * @note   文本 STOP/L STOP/R STOP 入口；可抢占 ENABLE/DISABLE/回滚序列
 */
void Aimotor_ControlStopStart(void);

/* 自检入口（AIMOTOR_SELF_TEST=1 时由 main 调用，实现在 aimotor_selftest.c） */

/**
 * @brief  固件自检：解析器/看门狗/电机安全/换算与 STATE/DRY_RUN 五组约 60 项
 * @note   仅 AIMOTOR_SELF_TEST=1（强制 DRY_RUN=1）时编译；结果经 USART1 输出
 */
void Aimotor_SelfTest(void);

/* Modbus 协议函数（参数化 bus_idx；定义见 aimotor_modbus.c） */

/** @brief Modbus RTU CRC16（poly 0xA001，初值 0xFFFF） */
uint16_t Aimotor_CRC16(const uint8_t *buf, uint16_t len);

/**
 * @brief  经总线发送 tx_buf 前 len 字节（TX LED 闪烁、等待发送完成）
 * @note   AIMOTOR_DRY_RUN=1 时改为 mock 记录，不做真实发送
 */
void     Aimotor_BusSendBytes(uint8_t bus_idx, uint16_t len);

/** @brief 伺服使能：06H 写 H03_03=1 */
void     Aimotor_ServoOn(uint8_t bus_idx, uint8_t slave_id);

/** @brief 伺服断开：06H 写 H03_03=0（与 ServoOn 同寄存器逆值） */
void     Aimotor_ServoOff(uint8_t bus_idx, uint8_t slave_id);

/** @brief 停止多段位运行：10H 写 H03_05=0 */
void     Aimotor_StopMotion(uint8_t bus_idx, uint8_t slave_id);

/**
 * @brief  写目标位置：10H 写 H11_12（Int32，CDAB 字节序）
 * @param  bus_idx 总线索引 0/1
 * @param  slave_id 从站号 1/2/3
 * @param  pulse 目标位置（脉冲），越界直接丢弃不发送
 */
void     Aimotor_SendPosition(uint8_t bus_idx, uint8_t slave_id,
                              int32_t pulse);

/** @brief 触发运行：10H 写 H03_05=1 */
void     Aimotor_TriggerMotion(uint8_t bus_idx, uint8_t slave_id);

/** @brief 查询实际位置：03H 读 H0B_07（2 寄存器，Int32 CDAB） */
void     Aimotor_QueryPosition(uint8_t bus_idx, uint8_t slave_id);

/** @brief 查询位置偏差：03H 读 H0B_15（编码器偏差计数，到位判断用） */
void     Aimotor_QueryPositionError(uint8_t bus_idx, uint8_t slave_id);

/* 上位机命令解析（定义见 host_protocol.c） */

/**
 * @brief  解析 B/ALL 文本命令（B1 M1 P10000 S300 / B1 STOP / ALL STOP 等）
 * @param  buf 命令文本（不含行结束符）
 * @param  len 文本长度
 * @param  cmd [out] 解析结果
 * @retval 1 有效命令；0 无效（部分格式错误经 USART1 回 ERR 提示）
 */
uint8_t  HostCmd_Parse(const uint8_t *buf, uint16_t len, Aimotor_Cmd_t *cmd);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __AIMOTOR_H__ */
