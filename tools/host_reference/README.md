# 上位机参考代码（旧架构，已归档）

`jaka_host_node_L_legacy.cpp` 是架构改造前的上位机 ROS 节点（`jaka_send_node_L`，左臂；
右臂为同构的 R 版本）：PC 通过 USB-RS485 直接驱动 AI 电机（Modbus RTU）、
通过串口直接驱动 MW 电机（0x3E 协议），订阅 `jaka_controller_tcp/Command`、
发布 `joint_states`，每臂一个节点实例。

该"PC 直驱电机"架构已被 **STM32 下位机集中驱动 + 二进制协议（AA 55 帧，见
`docs/API_Protocol_v1.0.md`）** 取代；本文件仅作协议历史与上位机集成方式参考。

**注意**：文件内 `LINEAR_FACTORS {4.0, 4.75, 2.0}` 的脉冲→长度换算已作废。
关节侧单位换算（µm/µrad）以下位机 `App/Src/aimotor.c` 的 `AiPulseToUm()` 为唯一事实来源，
上位机应直接消费 STATE 帧中的关节侧坐标，不得再做二次换算。
