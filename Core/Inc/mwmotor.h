/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mwmotor.h
  * @brief          : Moweidu motor private protocol (0x3E) module
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MWMOTOR_H__
#define __MWMOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define MW_BUS_COUNT      2       // USART6(L) + UART5(R)
#define MW_MOTORS_PER     3       // 每路 ID 4/5/6（J4/J5/J6）
#define MW_TICK_MS        5       // J4/J5/J6快速状态机调度周期
#define MW_RETRY_MAX      10      // 超时重试
#define MW_TIMEOUT_MS     20      // 串口超时
#define MW_RESPONSE_TIMEOUT_MS 30
#define MW_MOVE_TIMEOUT_MS 5000
#define MW_ARRIVE_SAMPLES 3
#define MW_ARRIVAL_TOLERANCE 200
/* 测试阶段暂不启用 J5/J6 到位判断；稳定后改为 1 并现场标定阈值。 */
#define MW_ENABLE_ARRIVAL_CHECK 0

/* 耦合关系常数 */
#define MW_COUPLE_J5      500     // J5 = motor5 × 100 / (5/3)  →  motor5 = J5 × 5/3 × 100
#define MW_COUPLE_J6      222     // J6 = (motor6+motor5) × 100 / (20/9) → motor6 = J6 × 20/9 × 100 - motor5

/* 关节角度限位：文档 J5/J6 范围 ±36000°，单位 deg×1000（1000 计数/度） */
#define MW_DEG1000_LIMIT  36000000LL

/* 空闲位置轮询间隔（tick 数，5ms/tick → 100ms） */
#define MW_IDLE_POLL_TICKS 20

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* ── 电机状态机步骤 ──────────────────────────────────────────── */
typedef enum {
    MW_STEP_IDLE    = 0,
    MW_STEP_WRITE   = 1,   // 发送 0xA3 位置命令
    MW_STEP_WAIT_WRITE = 2, // 等待0xA3响应
    MW_STEP_QUERY   = 3,   // 发送 0x92 读取角度
    MW_STEP_WAIT_RX = 4,   // 等待0x92回包
} MW_Step_t;

/* ── 单个电机 ────────────────────────────────────────────────── */
typedef struct {
    uint8_t  id;                // 站号 1~32
    int64_t  target_angle;      // 目标角度（兼容ROS：1000计数/度）
    uint32_t target_speed;      // 目标速度 (0.01dps/LSB)
    int64_t  actual_angle;      // 当前角度 (0x92 读取)
    MW_Step_t step;
    uint8_t  retry_count;
    uint8_t  cmd_pending;       // 上位机命令待处理
    uint8_t  actual_valid;      // actual_angle 来自最近成功回读
    uint8_t  enabled;           // 实际确认后的使能状态（0x88 确认=1，0x80 确认=0）
    uint8_t  fault;             // 该轴通信/执行故障
    uint32_t last_tx_tick;      // 上次发送时间（30ms 应答超时判据）
} MW_Motor_t;

/* J5/J6 是耦合关节，必须作为一个事务处理：
 * 读取 M5 -> 读取 M6 -> 计算/锁存目标 -> 写 M5 -> 写 M6。 */
typedef enum {
    MW_COUPLED_IDLE = 0,
    MW_COUPLED_READ_M5,
    MW_COUPLED_WAIT_M5,
    MW_COUPLED_READ_M6,
    MW_COUPLED_WAIT_M6,
    MW_COUPLED_WRITE_M5,
    MW_COUPLED_WAIT_WRITE_M5,
    MW_COUPLED_WRITE_M6,
    MW_COUPLED_WAIT_WRITE_M6,
} MW_CoupledStep_t;

typedef struct {
    MW_CoupledStep_t step;
    int64_t requested_j5_deg1000;
    int64_t requested_j6_deg1000;
    int64_t motor5_actual;
    int64_t motor6_actual;
    int64_t motor5_target;       // 当前正在执行的目标
    int64_t motor6_target;
    int64_t pending_motor5_target;
    int64_t pending_motor6_target;
    uint8_t pending;             // 有新的目标等待锁存
    uint8_t arrived;
    uint8_t arrive_count;
    uint8_t fault;
    uint32_t last_tx_tick;
    uint32_t move_start_tick;
    uint8_t retry_count;
} MW_CoupledTask_t;

/* ── 单路总线 ────────────────────────────────────────────────── */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *led_port;
    uint16_t      led_pin;
    uint8_t  tx_buf[64];
    uint8_t  rx_buf[64];
    volatile uint8_t  rx_ready;
    volatile uint16_t rx_len;
    uint8_t  current_motor;     // 轮转索引
} MW_Bus_t;

/* ── 全局变量 ────────────────────────────────────────────────── */
extern MW_Bus_t   mw_buses[MW_BUS_COUNT];
extern MW_Motor_t mw_motors[MW_BUS_COUNT][MW_MOTORS_PER];
extern MW_CoupledTask_t mw_coupled[MW_BUS_COUNT];

/* ── 命令结构体 ──────────────────────────────────────────────── */
typedef struct {
    uint8_t  side;          // 0=L, 1=R
    int16_t  j4_angle;      // J4 角度（度）, -1 表示不用
    int16_t  j5_angle;      // J5 角度（度）
    int16_t  j6_angle;      // J6 角度（度）
    uint16_t speed;         // 速度（°/s）
    uint8_t  valid;
} MW_Cmd_t;

/* ── API ─────────────────────────────────────────────────────── */
void MW_Init(void);
void MW_Process(void);
void MW_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
void MW_RecoverRx(UART_HandleTypeDef *huart);
/* 校验 MW 应答帧（0x88/0x80/0xA3/0x92 共用），供控制序列应答确认使用 */
uint8_t MW_ReplyOk(uint8_t bus_idx, uint8_t id, uint8_t cmd);
/* 重置空闲读取事务 owner（STOP/DISABLE/新事务开始） */
void MW_IdlePollReset(uint8_t side);
/* 冻结/恢复空闲位置轮询（自检用）：freeze=1 时暂停该总线空闲读 */
void MW_IdlePollSet(uint8_t side, uint8_t freeze);
/* 统一关节换算（文本/二进制共用，唯一基准，含范围检查）。
   输入为 deg×1000（1000 计数/度）；返回 0=成功，1=超范围/非法参数。
   m4/m5/m6 为电机侧目标（计数），已含右臂取反与 J5/J6 耦合。 */
uint8_t MW_ForwardKin(uint8_t side, int64_t j4_deg1000, int64_t j5_deg1000,
                      int64_t j6_deg1000, int64_t *m4, int64_t *m5, int64_t *m6);
/* 将已验证的换算结果写入电机状态（J4 独立 + J5/J6 耦合 pending） */
void MW_ApplyTargets(uint8_t side, int64_t j4_deg1000, int64_t j5_deg1000,
                     int64_t j6_deg1000, int64_t m4, int64_t m5, int64_t m6);
/* 二进制 TARGET 的 MW 侧换算+范围检查（不写状态，供原子校验） */
uint8_t MW_BinaryConvert(uint8_t side, const int32_t joint_urad[6],
                         int64_t *m4, int64_t *m5, int64_t *m6,
                         int64_t *j4_deg1000, int64_t *j5_deg1000,
                         int64_t *j6_deg1000);
/* 电机实际角度 → 关节侧 µrad（STATE 反馈用，与 MW_ForwardKin 互为逆运算） */
void MW_ReadbackUrad(uint8_t side, int64_t *j4_urad, int64_t *j5_urad,
                     int64_t *j6_urad);
void MW_HostCmd(const MW_Cmd_t *cmd);

/* 协议函数 */
uint8_t MW_Checksum(const uint8_t *buf, uint16_t len);
void    MW_SendBytes(uint8_t bus_idx, uint16_t len);
void    MW_SendA3(uint8_t bus_idx, uint8_t id, int64_t angle);
void    MW_Send92(uint8_t bus_idx, uint8_t id);
void    MW_Send80(uint8_t bus_idx, uint8_t id);  // 关闭
void    MW_Send88(uint8_t bus_idx, uint8_t id);  // 运行

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MWMOTOR_H__ */
