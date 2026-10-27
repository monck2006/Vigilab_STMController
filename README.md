# C1 控制系统设计 · 运动控制

实验室智能巡检机器人的 STM32 下位机固件。树莓派/ROS2 负责导航和决策；STM32 负责左右轮速度闭环、编码器里程和串口状态回报。

> 当前源码已按单一控制任务、20 ms 编码器/速度环、速度斜坡和明确故障边界重写。GCC 10.3.1 命令行构建已通过，产物位于 `C1_GCC/build/`；Keil 构建与真机验证尚未完成，编码器每轮计数和电机方向仍需上板确认。

## 目录

| 路径 | 内容 |
| --- | --- |
| `C1_Keil/` | Keil uVision 工程，ARM Compiler 6 |
| `C1_GCC/` | 命令行工程，arm-none-eabi-gcc |
| `docs/` | 固件架构、串口协议和 CubeMX 配置参考 |
| `.agent/` | 项目导览与自动控制原理说明 |
| `tools/` | Python 协议实现、串口测试、里程标定和辅助工具 |
| `release/` | 仓库中已有的发布镜像；不代表它由当前源码生成 |

Keil 和 GCC 工程共用 `C1_Keil/Core` 中的控制源码。

## 控制流程

```text
USART3 收命令 → Control 任务 → 目标速度斜坡 → 速度 PI → TIM3 PWM
                                      ▲                    │
                               TIM2/TIM8 编码器 ← 电机 ← BTS7960

状态/ACK 队列 → Telemetry 任务 → USART3
```

`Task_Control` 统一处理命令、安全状态、编码器测速和 PI，避免多个任务同时修改电机状态。目标速度按默认 300 mm/s² 的斜坡变化；速度环默认 20 ms（50 Hz）。

## 开始阅读

推荐顺序：

1. `.agent/项目导览.md`：项目边界和文件地图。
2. `.agent/方案与代码逐步讲解.md`：完整方案、代码流程和上板步骤。
3. `.agent/自动控制入门与改造指南.md`：闭环、PI、编码器换算的入门说明。
4. `C1_Keil/Core/Inc/config.h`：硬件比例、方向和控制参数。
5. `C1_Keil/Core/Src/app_tasks.c`：命令处理、安全逻辑和速度控制。
6. `encoder.c`、`motor.c`、`protocol.c`：测量、执行与通信。

## 关键硬件参数和待校准项

- MCU：启明欣欣 STM32F407ZGT6 高配版 V6.1。
- 电机驱动：BTS7960，逻辑 VCC 与电机电源按模块接线区分，STM32 与驱动模块共地。
- 轮径 180 mm，轮距 428 mm（按机械设计资料）。
- 编码器资料标称 16 PPR，但未说明轴侧。`config.h` 暂按 `16 × 4 × 56 = 3584` 计数/车轮转；需逐轮手动转一圈实测后再标定。
- 默认编码器方向和电机方向符号均为 `+1`；架空低速测试时应确认前进命令对应左右正速度。
- 默认不使用 PA5/PF11 控制 EN，不把 PA0 当电源开关。BTS7960 EN 按实际模块接线拉高；PA0 是 UART4_TX 引出。
- PWM 的 PB4/PB5/PB0/PB1 与开发板 Flash/触摸屏复用；编码器 PA15/PB3 与 JTAG 复用；PC6/PC7 与 USART6 复用。调试使用 SWD，并关闭相关冲突外设。
- 当前没有电流/电压 ADC；状态帧母线电压使用 `0xFFFF` 表示未知。堵转检测默认关闭。软件急停不能替代硬件断电急停。

## 编译入口

GCC 10.3.1 命令行构建已通过。Keil 工程尚未在本机验证；准备好对应工具链后可分别构建：

```powershell
# Keil uVision
& "D:\keil5\UV4\UV4.exe" -b C1_Keil\Project\C1.uvprojx -o build.log

# GCC 命令行
cd C1_GCC
powershell -ExecutionPolicy Bypass -File build.ps1
```

## 串口命令

USB 转 TTL 连接 USART3（PB10=TX、PB11=RX），共地，3.3 V TTL，默认 115200 8N1。电脑端工具示例：

```powershell
python tools\serial_tester.py -p COM3 monitor
python tools\serial_tester.py -p COM3 drive 50 50
python tools\serial_tester.py -p COM3 pid 0.3 1.0
python tools\serial_tester.py -p COM3 stop
python tools\serial_tester.py -p COM3 estop
python tools\serial_tester.py -p COM3 clear
```

完整帧字段和错误码见 `docs/02_上下位机通信协议.md`。第一次发运动命令前，先架空车轮并把目标速度设得很低。
`speed` 只发一帧，适合观察通信超时软停；`drive` 每 100 ms 续发速度命令，按 Ctrl+C 会请求软停。
