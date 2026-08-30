# STM32 双臂下位机安全返修说明（v1.3）

基于上一版 `review_v6/AIMotor_F407_Demo` 修改，原工程未覆盖。

## 本次修复

1. 已处于 `CONTROL_ENABLED` 时，重复二进制 `ENABLE` 改为立即幂等 ACK：
   - 12 轴均确认使能且无轴故障：回 `0x00` 并刷新 250 ms 看门狗；
   - 轴状态不一致或存在锁存故障：回 `0x0A`；
   - 不再重跑 12 轴使能序列，消除看门狗长时间屏蔽窗口。
2. AI/MW 普通运动通信故障锁存：新 `TARGET` 不会清故障自动重启；成功
   `ENABLE` 才清除可恢复轴故障并恢复轴状态机。
3. `B STOP`、`ALL STOP`、`L/R STOP` 均可抢占 ENABLE/DISABLE/ROLLBACK，
   并统一执行双臂 12 轴全局安全停止。
4. 永久禁用会直通 Servo On 的文本 `B TEST`；文本 `GO` 改由生产异步状态机
   执行，不再累计调用 100 ms `HAL_Delay()`。
5. 修复主机协议脚本：STATE `axis_status` 使用 uint32，STOP 的 arm 兼容字段
   改为任意值均接受。
6. 修复固件自检：
   - 测试间复位 `g_test_reply_wait`；
   - R_J3 enabled 位改为 bit8 (`0x0100`)；
   - fuzz 使用实际完成轮数与缓冲有界条件；
   - 回滚通过生产发送入口验证 18 步及全部轴掩码；
   - 增加重复 ENABLE 后断联、轴故障锁存、文本 STOP 抢占测试；
   - DRY_RUN 测试前清空 mock 日志。
7. 配置宏增加 `#ifndef`，可由 Keil/CubeIDE 编译配置覆盖，便于构建
   `DRY_RUN=1 + SELF_TEST=1` 测试版本。
8. 同步协议、架构和 HTML API 文档。

## 已执行验证

- `python3 tools/protocol_test.py`：`60 passed, 0 failed`，退出码 0。
- 发布配置（`DRY_RUN=0, SELF_TEST=0`）全部 `Core/Src` C 语法检查：退出码 0。
- 自检配置（`DRY_RUN=1, SELF_TEST=1`）全部 `Core/Src` C 语法检查：退出码 0。

当前环境没有 Keil ARMClang，因此未生成新的 `.hex/.axf`，也没有在 STM32 板上
运行 C 自检。压缩包内不应使用旧版构建目录中的固件；请在 Keil 中重新构建。

## 建议烧录前步骤

1. 先用 `AIMOTOR_DRY_RUN=1`、`AIMOTOR_SELF_TEST=1` 构建并烧录，确认 USART1
   最终打印 `0 failed`。
2. 再恢复发布配置 `AIMOTOR_DRY_RUN=0`、`AIMOTOR_SELF_TEST=0` 全量重编译。
3. 电机断电抓包验证重复 ENABLE、STOP 抢占和 250 ms 看门狗。
4. 通过上述步骤后再进行带动力低速、小位移测试。
