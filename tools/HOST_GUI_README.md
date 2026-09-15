# 图形化上位机

用于通过 USART1 测试 STM32F407 下位机正式二进制协议。

## 安装与运行

Ubuntu：

```bash
sudo apt install python3-tk
python3 -m pip install pyserial
python3 tools/host_gui.py
```

Windows：

```powershell
py -m pip install pyserial
py tools\host_gui.py
```

串口为 `115200 / 8N1`。接线：

```text
USB-TTL TX -> STM32 PA10 (USART1 RX)
USB-TTL RX <- STM32 PA9  (USART1 TX)
USB-TTL GND -- STM32 GND
```

## 首次测试

1. 连接并确认日志出现 `HELLO: 成功`。
2. 保持自动刷新开启，确认状态与关节反馈持续更新。
3. 点击“使能”，确认状态变为 `ENABLED`。
4. 选择机械臂，点击“填入当前位置”。
5. 只小幅修改一个轴，再发送 TARGET。
6. 测试“全局停止”和“失能”。

自动刷新周期为 100 ms，也用于满足固件在 ENABLED 状态下的 250 ms 通信看门狗。
J1~J3 单位为 µm，J4~J6 单位为 µrad。
