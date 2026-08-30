# AIMotor_F407_Demo — 双机械臂 12 轴电机下位机

基于 STM32F407VET6 的双机械臂下位机固件。4 路 UART/RS485 总线驱动 12 个电机
（6 个 AI 电机：Modbus RTU、脉冲制；6 个慕纬度电机：私有协议 0x3E、角度制、J5/J6 耦合），
通过 USART1 与上位机（ROS/MoveIt）通信，支持文本命令与二进制协议（AA 55 帧 + CRC16，
见 `docs/API_Protocol_v1.0.md`）。

## 目录结构

```
AIMotor_F407_Demo/
├── AIMotor_F407_Demo.ioc   # CubeMX 工程文件
├── Core/                    # CubeMX 生成区（main/usart/dma/gpio 等，勿手工改）
├── App/                     # 手写应用代码（CubeMX 重新生成不覆盖）
│   ├── Src/aimotor.c        # AI 电机 Modbus RTU + 状态机 + 上位机流解析/控制序列
│   ├── Src/mwmotor.c        # MW 电机 0x3E 协议 + 状态机 + J5/J6 耦合解算
│   └── Inc/                 # 对应头文件
├── Drivers/                 # STM32F4 HAL + CMSIS（CubeMX 管理）
├── MDK-ARM/                 # Keil 工程（构建产物已被 .gitignore 排除）
├── docs/                    # 架构 / 协议 / 驱动 / 返修记录
└── tools/
    ├── protocol_test.py     # 主机端协议回归测试（python3 tools/protocol_test.py）
    └── host_reference/      # 旧"PC 直驱电机"架构的上位机参考代码（已归档）
```

## 构建

Keil MDK-ARM V5 + ARMCLANG：打开 `MDK-ARM/AIMotor_F407_Demo.uvprojx` 直接编译。
应用代码在 `Application/User/App` 组；`Core/`、`Drivers/` 由 CubeMX 重新生成时自动管理。

固件自检（无硬件干跑）：编译配置设 `AIMOTOR_DRY_RUN=1` + `AIMOTOR_SELF_TEST=1`，
上电后经 USART1 输出逐项 PASS/FAIL。

## 文档索引

- `docs/Project_Architecture.md` — 系统架构、数据流、状态机设计
- `docs/API_Protocol_v1.0.md` — 上位机二进制协议规范
- `docs/README_J456_DRIVER.md` — J4/J5/J6 驱动说明
- `docs/MODIFICATION_REPORT_v1.3.md` — v1.3 安全返修记录
