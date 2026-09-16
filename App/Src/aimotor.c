/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : aimotor.c
  * @brief          : AI Motor motor core (arrays/init/DMA routing/state machine)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "aimotor.h"
#include "mwmotor.h"
#include "usart.h"
#include "aimotor_internal.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* ======================================================================== */
/*               编译期配置确认（打印到编译日志，防止宏未生效）                  */
/* ======================================================================== */

#define AIMOTOR_XSTR_(x) #x
#define AIMOTOR_XSTR(x)  AIMOTOR_XSTR_(x)
#pragma message("AIMOTOR_DRY_RUN             = " AIMOTOR_XSTR(AIMOTOR_DRY_RUN))
#pragma message("AIMOTOR_TEXT_MOTION_ENABLED = " AIMOTOR_XSTR(AIMOTOR_TEXT_MOTION_ENABLED))
#pragma message("AIMOTOR_SELF_TEST           = " AIMOTOR_XSTR(AIMOTOR_SELF_TEST))

#if AIMOTOR_SELF_TEST && !AIMOTOR_DRY_RUN
#error "AIMOTOR_SELF_TEST requires AIMOTOR_DRY_RUN=1"
#endif

/* ======================================================================== */
/*                          全局变量定义                                      */
/* ======================================================================== */

Aimotor_Bus_t aimotor_buses[AIMOTOR_BUS_COUNT];
Aimotor_t     aimotor_motors[AIMOTOR_BUS_COUNT][AIMOTOR_MOTORS_PER];

/* 12 轴轴序静态断言：L_J1..L_J6, R_J1..R_J6（位图统一 side*6+joint） */
_Static_assert(AIMOTOR_BUS_COUNT * (AIMOTOR_MOTORS_PER + MW_MOTORS_PER) == 12,
               "12 axis bitmap requires 2 arms x 6 joints");

/* DRY_RUN 计数与 mock 记录（真实发送计数：DRY_RUN 下恒为 0） */
volatile uint32_t g_dry_run_ai_tx_count = 0;
volatile uint32_t g_dry_run_mw_tx_count = 0;
volatile DryRunLogEntry_t g_dry_run_log[AIMOTOR_DRY_RUN_LOG_MAX];
volatile uint8_t  g_dry_run_log_index = 0;

/* mock：记录“将发送”的命令帧（DRY_RUN 下真实发送函数被短路） */
void Aimotor_DryRunLog(uint8_t is_mw, const uint8_t *data, uint16_t len)
{
    volatile DryRunLogEntry_t *e = &g_dry_run_log[g_dry_run_log_index];
    uint16_t n = (len < AIMOTOR_DRY_RUN_FRAME_MAX) ? len : AIMOTOR_DRY_RUN_FRAME_MAX;
    for (uint16_t i = 0; i < n; i++) e->data[i] = data[i];
    e->len = (uint8_t)n;
    e->is_mw = is_mw;
    g_dry_run_log_index = (uint8_t)((g_dry_run_log_index + 1U) %
                                    AIMOTOR_DRY_RUN_LOG_MAX);
}

static uint8_t Aimotor_RequestTimedOut(const Aimotor_t *motor)
{
    return (uint8_t)((HAL_GetTick() - motor->last_tx_tick) >=
                     AIMOTOR_RESPONSE_TIMEOUT_MS);
}

/* ======================================================================== */
/*                       回包解析处理                                       */
/* ======================================================================== */

static uint8_t ProcessRx(uint8_t bus_idx, uint8_t motor_idx)
{
    Aimotor_t *motor = &aimotor_motors[bus_idx][motor_idx];
    uint8_t ok = 0;

    motor->rx_ready = 0;

    /* 先检查长度，避免 rx_len - 2 下溢 */
    if (motor->rx_len < 5U) {
        return 0;
    }

    /* CRC 校验 */
    uint16_t calc_crc = Aimotor_CRC16(motor->rx_buf, motor->rx_len - 2);
    uint16_t rec_crc  = (motor->rx_buf[motor->rx_len - 1] << 8)
                      |  motor->rx_buf[motor->rx_len - 2];
    if (calc_crc != rec_crc) {
        return 0;
    }

    if (motor->rx_buf[0] != motor->slave_id) {
        return 0;
    }

    switch (motor->step) {
    case MOTOR_STEP_WAIT_STOP:
        ok = (motor->rx_len == 8U && motor->rx_buf[1] == 0x10U &&
              motor->rx_buf[2] == 0x03U && motor->rx_buf[3] == 0x05U &&
              motor->rx_buf[4] == 0x00U && motor->rx_buf[5] == 0x01U);
        break;
    case MOTOR_STEP_WAIT_WRITE:
        ok = (motor->rx_len == 8U && motor->rx_buf[1] == 0x10U &&
              motor->rx_buf[2] == 0x11U && motor->rx_buf[3] == 0x0CU &&
              motor->rx_buf[4] == 0x00U && motor->rx_buf[5] == 0x02U);
        break;
    case MOTOR_STEP_WAIT_TRIGGER:
        ok = (motor->rx_len == 8U && motor->rx_buf[1] == 0x10U &&
              motor->rx_buf[2] == 0x03U && motor->rx_buf[3] == 0x05U &&
              motor->rx_buf[4] == 0x00U && motor->rx_buf[5] == 0x01U);
        break;
    case MOTOR_STEP_WAIT_QUERY:
        /* H0B_07绝对位置 */
        if (motor->rx_len == 9U && motor->rx_buf[1] == 0x03U &&
            motor->rx_buf[2] == 0x04U) {
            motor->actual_position =
                Aimotor_DecodeInt32CDAB(&motor->rx_buf[3]);
            motor->actual_valid = 1;
            ok = 1;
        }
        break;
    case MOTOR_STEP_WAIT_ERROR:
        /* H0B_15编码器位置偏差 */
        if (motor->rx_len == 9U && motor->rx_buf[1] == 0x03U &&
            motor->rx_buf[2] == 0x04U) {
            motor->position_error =
                Aimotor_DecodeInt32CDAB(&motor->rx_buf[3]);
            ok = 1;
        }
        break;
    default:
        break;
    }

    if (ok) {
        motor->retry_count = 0;
    }
    return ok;
}

/* ======================================================================== */
/*                         系统初始化                                       */
/* ======================================================================== */

void Aimotor_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    uint8_t b, m;

    /* ── PB0 LED 初始化：总线 #1 (USART2) TX 指示 ────────────────── */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Pin = GPIO_PIN_0;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    /* ── PB2 LED 初始化：总线 #2 (USART3) TX 指示 ────────────────── */
    gpio_init.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    /* ── 初始化总线 #0: USART2 (PA2/PA3), PB0 LED ───────────────── */
    aimotor_buses[0].huart    = &huart2;
    aimotor_buses[0].led_port = GPIOB;
    aimotor_buses[0].led_pin  = GPIO_PIN_0;
    aimotor_buses[0].rx_ready = 0;
    aimotor_buses[0].rx_len   = 0;
    aimotor_buses[0].current_motor = 0;

    /* ── 初始化总线 #1: USART3 (PB10/PB11), PB2 LED ─────────────── */
    aimotor_buses[1].huart    = &huart3;
    aimotor_buses[1].led_port = GPIOB;
    aimotor_buses[1].led_pin  = GPIO_PIN_2;
    aimotor_buses[1].rx_ready = 0;
    aimotor_buses[1].rx_len   = 0;
    aimotor_buses[1].current_motor = 0;

    /* ── 初始化每总线的 3 个电机 ────────────────────────────────── */
    for (b = 0; b < AIMOTOR_BUS_COUNT; b++) {
        for (m = 0; m < AIMOTOR_MOTORS_PER; m++) {
            aimotor_motors[b][m].slave_id       = m + 1;
            aimotor_motors[b][m].target_position = 0;
            aimotor_motors[b][m].actual_position = 0;
            aimotor_motors[b][m].position_error = 0;
            aimotor_motors[b][m].step            = MOTOR_STEP_IDLE;
            aimotor_motors[b][m].retry_count     = 0;
            aimotor_motors[b][m].cmd_pending     = 0;
            aimotor_motors[b][m].enabled         = 0;
            aimotor_motors[b][m].arrived         = 0;
            aimotor_motors[b][m].arrive_count    = 0;
            aimotor_motors[b][m].fault           = 0;
            aimotor_motors[b][m].last_tx_tick    = 0;
            aimotor_motors[b][m].move_start_tick = HAL_GetTick();
            aimotor_motors[b][m].actual_valid    = 0;
        }
    }

    /* ── 启动各 RS485 总线的 DMA IDLE 接收 ───────────────────────── */
    HAL_UARTEx_ReceiveToIdle_DMA(aimotor_buses[0].huart,
                                  aimotor_buses[0].rx_buf, AIMOTOR_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(aimotor_buses[1].huart,
                                  aimotor_buses[1].rx_buf, AIMOTOR_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(aimotor_buses[0].huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(aimotor_buses[1].huart->hdmarx, DMA_IT_HT);

    /* ── 上电安全停止序列（异步，不阻塞） ──────────────────────────
       复位后向全部 AI 电机发送真实 Servo Off（0x0303=0），向全部 MW 电机
       发送 0x80 关闭，清除全部待执行目标；序列完成前控制状态保持
       DISABLED，完成后仍为 DISABLED。绝不进入 ENABLED，绝不自动恢复
       复位前目标。序列由主循环 CtrlSeqTick 推进。 */
    g_ctrl_seq.kind = CTRL_SEQ_NONE;
    g_ctrl_seq.active = 0;
    CtrlSeqStart(CTRL_SEQ_STARTUP_SAFE_STOP, 0, 0, 0);
}

/* ======================================================================== */
/*                        DMA IDLE 回调路由                                */
/* ======================================================================== */

void Aimotor_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t i, m;

    /* Size 为 0 时重新启动 DMA 后直接返回 */
    if (Size == 0U) {
        for (i = 0; i < AIMOTOR_BUS_COUNT; i++) {
            if (aimotor_buses[i].huart == huart) {
                HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                              aimotor_buses[i].rx_buf,
                                              AIMOTOR_BUF_SIZE);
                __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
                return;
            }
        }
        return;
    }

    /* 按从站号路由到对应电机私有缓冲 */
    for (i = 0; i < AIMOTOR_BUS_COUNT; i++) {
        if (aimotor_buses[i].huart == huart) {
            uint8_t slave = aimotor_buses[i].rx_buf[0];
            for (m = 0; m < AIMOTOR_MOTORS_PER; m++) {
                if (aimotor_motors[i][m].slave_id == slave) {
                    uint16_t copy_len = Size;
                    if (copy_len > sizeof(aimotor_motors[i][m].rx_buf)) {
                        copy_len = sizeof(aimotor_motors[i][m].rx_buf);
                    }

                    memcpy(aimotor_motors[i][m].rx_buf,
                           aimotor_buses[i].rx_buf,
                           copy_len);

                    /* rx_len 字段为 uint8_t，本处最大 16，转换安全 */
                    aimotor_motors[i][m].rx_len = (uint8_t)copy_len;
                    aimotor_motors[i][m].rx_ready = 1;
                    break;
                }
            }
            HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                          aimotor_buses[i].rx_buf,
                                          AIMOTOR_BUF_SIZE);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

void Aimotor_RecoverRx(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < AIMOTOR_BUS_COUNT; ++i) {
        if (aimotor_buses[i].huart == huart) {
            HAL_UART_AbortReceive(huart);
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            aimotor_buses[i].rx_ready = 0;
            aimotor_buses[i].rx_len = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                          aimotor_buses[i].rx_buf,
                                          AIMOTOR_BUF_SIZE);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

/* ======================================================================== */
/*                        主状态机调度                                     */
/* ======================================================================== */

void Aimotor_Process(void)
{
    static uint8_t round_robin = 0;   /* 轮转索引 0/1/2 */
    uint8_t b;
    Aimotor_t *motor;

    /* 上位机二进制流解析（拆包/粘包/CRC 重同步 + 文本行提取） */
    Aimotor_HostStreamPoll();

    /* 异步控制序列推进（ENABLE/STOP/DISABLE/上电安全停止）。
       序列进行期间暂停 AI 轮转与 MW 状态机，序列独占两族总线，
       避免同一条 RS485 总线上同时存在两个未完成请求。 */
    CtrlSeqTick();

    /* ── 处理上位机命令（非阻塞轮询, 只消费 B/ALL 命令） ──────────
       控制序列活动时只允许文本 STOP 穿透并抢占；其它文本命令继续排队。 */
    if (g_host_cmd_ready) {
        /* 判断首字符: B→AI电机, A→ALL, L/R→MW电机(不消费) */
        uint8_t c = g_host_cmd_buf[0];
        if (c == 'B' || c == 'b' || c == 'A' || c == 'a') {
            uint8_t is_stop = (strstr(g_host_cmd_buf, "STOP") != NULL ||
                               strstr(g_host_cmd_buf, "stop") != NULL);
            if (!Aimotor_CtrlSeqActive() || is_stop) {
                g_host_cmd_ready = 0;
                Aimotor_Cmd_t cmd;
                if (HostCmd_Parse((const uint8_t *)g_host_cmd_buf,
                                  g_host_cmd_len, &cmd)) {
                    HostCmd_Execute(&cmd);
                }
            }
        }
    }

    if (Aimotor_CtrlSeqActive()) {
        return;   /* 序列进行中：暂停 AI/MW 状态机，不接收新目标 */
    }

    /* ── 轮转调度：每个tick每条总线处理一个电机 ─────────────────── */
    for (b = 0; b < AIMOTOR_BUS_COUNT; b++) {
        /* 总线唯一事务 owner：若该总线存在未完成请求（任一电机处于 WAIT_*），
           本轮只处理该电机（回包+超时），不启动其它电机的新事务；
           保证一条 RS485 总线上同一时刻只有一个未完成请求，回包不会
           因轮转跳过而滞留或产生总线命令重叠。 */
        uint8_t owner = 0xFF;
        for (uint8_t m = 0; m < AIMOTOR_MOTORS_PER; m++) {
            Aimotor_Step_t st = aimotor_motors[b][m].step;
            if (st == MOTOR_STEP_WAIT_STOP || st == MOTOR_STEP_WAIT_WRITE ||
                st == MOTOR_STEP_WAIT_TRIGGER || st == MOTOR_STEP_WAIT_QUERY ||
                st == MOTOR_STEP_WAIT_ERROR) {
                owner = m;
                break;
            }
        }
        motor = (owner != 0xFF) ? &aimotor_motors[b][owner]
                                : &aimotor_motors[b][round_robin];
        uint8_t mi = (uint8_t)(motor - aimotor_motors[b]);
        uint8_t defer_next_tx = 0U;

        /* 处理当前请求的回包；无效回包会进入超时重试路径。 */
        if (motor->rx_ready &&
            (motor->step == MOTOR_STEP_WAIT_STOP ||
             motor->step == MOTOR_STEP_WAIT_WRITE ||
             motor->step == MOTOR_STEP_WAIT_TRIGGER ||
             motor->step == MOTOR_STEP_WAIT_QUERY ||
             motor->step == MOTOR_STEP_WAIT_ERROR)) {
            if (ProcessRx(b, mi)) {
                switch (motor->step) {
                case MOTOR_STEP_WAIT_STOP:
                    motor->step = MOTOR_STEP_WRITE;
                    defer_next_tx = 1U;
                    break;
                case MOTOR_STEP_WAIT_WRITE:
                    motor->step = MOTOR_STEP_TRIGGER;
                    defer_next_tx = 1U;
                    break;
                case MOTOR_STEP_WAIT_TRIGGER:
#if AIMOTOR_ENABLE_ARRIVAL_CHECK
                    /* 到位判断：直连偏差查询（0x0B15），跳过 0x0B07 位置读取 */
                    motor->step = MOTOR_STEP_QUERY_ERROR;
#else
                    /* 触发已确认，运动序列结束（QUERY 移交空闲轮询：
                       每次运动省一次总线往返；手册 §5.2 要求每次运动前
                       重新导通多段位使能，故 STOP 步不可省） */
                    motor->step = MOTOR_STEP_IDLE;
#endif
                    break;
                case MOTOR_STEP_WAIT_QUERY:
                    /* 本步仅空闲轮询（IDLE→QUERY）进入：缓存位置并置 actual_valid */
#if AIMOTOR_ENABLE_ARRIVAL_CHECK
                    motor->step = MOTOR_STEP_QUERY_ERROR;
#else
                    motor->step = MOTOR_STEP_IDLE;
#endif
                    break;
                case MOTOR_STEP_WAIT_ERROR:
#if AIMOTOR_ENABLE_ARRIVAL_CHECK
                    if (motor->position_error <=
                            AIMOTOR_POS_ERROR_TOLERANCE_ENCODER &&
                        motor->position_error >=
                            -AIMOTOR_POS_ERROR_TOLERANCE_ENCODER) {
                        if (motor->arrive_count < AIMOTOR_ARRIVE_SAMPLES)
                            motor->arrive_count++;
                    } else {
                        motor->arrive_count = 0;
                    }
                    motor->arrived =
                        (motor->arrive_count >= AIMOTOR_ARRIVE_SAMPLES);
                    if (motor->cmd_pending) {
                        motor->step = MOTOR_STEP_STOP;
                    } else if (motor->arrived) {
                        motor->step = MOTOR_STEP_ARRIVED;
                    } else if ((HAL_GetTick() - motor->move_start_tick) >=
                               AIMOTOR_MOVE_TIMEOUT_MS) {
                        motor->fault = 1;
                        motor->step = MOTOR_STEP_FAULT;
                    } else {
                        motor->step = MOTOR_STEP_QUERY;
                    }
#else
                    /* 到位检测暂时注释，保留 H0B-15 回包解析供后续启用。 */
                    motor->step = MOTOR_STEP_IDLE;
#endif
                    break;
                default:
                    break;
                }
            }
        }

        /*
         * 必须在处理查询回包之后接收新目标。
         * 否则 WAIT_QUERY 刚变成 IDLE 就会在下方重新进入 QUERY，
         * cmd_pending 在持续轮询期间将一直得不到执行。
         */
        if (motor->cmd_pending &&
            (motor->step == MOTOR_STEP_IDLE ||
             motor->step == MOTOR_STEP_ARRIVED) &&
            !motor->fault) {
            motor->cmd_pending = 0;
            motor->retry_count = 0;
            motor->arrived = 0;
            motor->arrive_count = 0;
            motor->actual_valid = 0;   /* 新目标执行期间反馈视为非最新 */
            motor->move_start_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_STOP;
        }

        /* 驱动器确认 STOP/WRITE 后，等到下一次 5ms 调度再发送后续命令。
           保留旧版已验证流程中的命令间隔，避免同一轮紧接着写下一寄存器。 */
        if (defer_next_tx) {
            continue;
        }

        switch (motor->step) {

        case MOTOR_STEP_IDLE:
            /* 空闲时保持位置和位置偏差缓存最新。 */
            motor->step = MOTOR_STEP_QUERY;
            break;

        case MOTOR_STEP_STOP:
            motor->rx_ready = 0;
            Aimotor_StopMotion(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            /* 重试计数只在事务启动时清零；重发时保留，确保达到上限后停发 */
            motor->step = MOTOR_STEP_WAIT_STOP;
            break;

        case MOTOR_STEP_WRITE:
            motor->rx_ready = 0;
            Aimotor_SendPosition(b, motor->slave_id,
                                 motor->target_position);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_WRITE;
            break;

        case MOTOR_STEP_TRIGGER:
            motor->rx_ready = 0;
            Aimotor_TriggerMotion(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_TRIGGER;
            break;

        case MOTOR_STEP_QUERY:
            motor->rx_ready = 0;
            Aimotor_QueryPosition(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_QUERY;
            break;

        case MOTOR_STEP_QUERY_ERROR:
            motor->rx_ready = 0;
            Aimotor_QueryPositionError(b, motor->slave_id);
            motor->last_tx_tick = HAL_GetTick();
            motor->step = MOTOR_STEP_WAIT_ERROR;
            break;

        case MOTOR_STEP_WAIT_STOP:
        case MOTOR_STEP_WAIT_WRITE:
        case MOTOR_STEP_WAIT_TRIGGER:
        case MOTOR_STEP_WAIT_QUERY:
        case MOTOR_STEP_WAIT_ERROR:
            if (Aimotor_RequestTimedOut(motor)) {
                motor->rx_ready = 0;
                if (++motor->retry_count > AIMOTOR_RETRY_MAX) {
                    motor->retry_count = 0;
                    motor->fault = 1;
                    motor->step = MOTOR_STEP_FAULT;
                } else {
                    switch (motor->step) {
                    case MOTOR_STEP_WAIT_STOP:    motor->step = MOTOR_STEP_STOP; break;
                    case MOTOR_STEP_WAIT_WRITE:   motor->step = MOTOR_STEP_WRITE; break;
                    case MOTOR_STEP_WAIT_TRIGGER: motor->step = MOTOR_STEP_TRIGGER; break;
                    case MOTOR_STEP_WAIT_QUERY:   motor->step = MOTOR_STEP_QUERY; break;
                    case MOTOR_STEP_WAIT_ERROR:   motor->step = MOTOR_STEP_QUERY_ERROR; break;
                    default: break;
                    }
                }
            }
            break;

        case MOTOR_STEP_ARRIVED:
            if (motor->cmd_pending) {
                motor->step = MOTOR_STEP_STOP;
            }
            break;

        case MOTOR_STEP_FAULT:
            /* 故障锁存：TARGET 校验会拒绝故障轴；仅成功 ENABLE 清除并恢复 IDLE。 */
            motor->cmd_pending = 0;
            break;

        default:
            motor->step = MOTOR_STEP_FAULT;
            motor->fault = 1;
            break;
        }
    }

    /* 下一轮转索引 */
    round_robin = (round_robin + 1) % AIMOTOR_MOTORS_PER;
}

/* USER CODE END 0 */
