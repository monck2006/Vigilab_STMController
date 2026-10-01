# C1 Keil 工程说明

双击 `Project\C1.uvprojx` 用 Keil uVision 打开。

## 工程内容

| 分组 | 内容 |
| --- | --- |
| Core | `main.c`（时钟与外设初始化）、`stm32f4xx_it.c`（中断）、`stm32f4xx_hal_msp.c`（引脚/时钟/DMA）、`protocol.c`、`motor.c`、`encoder.c`、`app_tasks.c` |
| CMSIS | 启动文件与系统时钟 |
| STM32F4xx_HAL_Driver | 只挑了用得到的 17 个 HAL 源文件 |
| FreeRTOS | 内核 + GCC 端口的 Cortex-M4F 移植 + heap_4 |

## 关键编译选项（已配好）

| 选项 | 值 | 原因 |
| --- | --- | --- |
| 编译器 | ARM Compiler 6（armclang，V6.21） | 你们装的是 AC6，没有 AC5 |
| C 标准 | gnu11 | 代码里用了 C99 写法 |
| 浮点 | `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` | 速度环用得到硬件浮点，FreeRTOS 端口也要求开 |
| 优化 | -O2 | 控制环性能与体积平衡 |
| 预定义宏 | `USE_HAL_DRIVER, STM32F407xx` | HAL 需要 |
| FreeRTOS 端口 | `portable/GCC/ARM_CM4F` | FreeRTOS 官方说明：AC6 必须用 GCC 端口（见 `portable/ARMClang/Use-the-GCC-ports.txt`） |

## 编译结果（已验证）

```
Program Size: Code=22450  RO-data=478  RW-data=16  ZI-data=27344
".\OBJ\C1.axf" - 0 Error(s)
FromELF: creating hex file...
```

镜像 **22944 字节**，在免费版 MDK-Lite 的 32KB 限制之内，**不需要任何许可就能编译**，产物在 `Project\OBJ\`：

| 文件 | 说明 |
| --- | --- |
| `C1.axf` | 带调试信息的可执行文件，Keil 里点 Download 直接烧录 |
| `C1.hex` | 可交给任何烧录工具（STM32CubeProgrammer、DAP 下载器等） |

### 为什么能塞进 32KB

关键选项是 **"One ELF Section per Function"（每个函数单独成段）**：

| 选项 | 不开 | 打开 |
| --- | --- | --- |
| 镜像大小 | 50120 字节（超限报 L6050U） | **22944 字节** |

打开后 armlink 能把没用到的 HAL 函数整段丢掉，体积直接砍掉一半多。这个开关已在工程里配好（`<OneElfS>1</OneElfS>`），不要关掉，否则又会超出免费版限制。

## 备用的 GCC 工程

如果以后需要脱离 Keil 编译，`..\C1_GCC\` 下有一套等价的命令行工程（arm-none-eabi-gcc 13.3.1，已在 `D:\xpack-arm-none-eabi-gcc-13.3.1-1.1`），跑 `build.ps1` 即可：

```
C1.elf / C1.hex / C1.bin   —— FLASH 占用 21200 字节，RAM 27312 字节
```

两套工程用的是同一份源码（`C1_Keil\Core`），改一处两边都能编。

## 烧录

用 CMSIS-DAP（你们资料包里那个仿真器）或 ST-Link 都行，SWD 四线：SWDIO / SWCLK / GND / 3.3V。
注意**目标板要单独供电**，仿真器不供 12V。

工程的调试器配置（来自厂商例程）已经带过来了：CMSIS-DAP + **SW-DP**，下载算法 STM32F4xx_1024。

### 首次烧录检查清单

| # | 检查项 | 原因 |
| --- | --- | --- |
| 1 | 调试口用 **SWD，不能用 JTAG** | PA15/PB3 被用作编码器输入，JTAG 会占用这两个脚 |
| 2 | **先不接电机**（或断掉电机电源） | 虽然固件上电时 EN 拉低、PWM 为 0，仍建议先空载验证通信 |
| 3 | LCD/触摸屏若插着，建议拔掉 | EN 用的 PA5/PF11 是触摸屏引脚；或把 `config.h` 的 `MOTOR_USE_EN` 改 0，EN 直接接 3.3V |
| 4 | 板载 W25Q128 的片选 PG8 已由固件拉高 | 它的 DO 接在 PB4 上（我们的 PWM），不拉高会有总线冲突 |

### 烧完之后怎么判断在跑

固件没有做 LED 指示，判断方法是**看串口**：打开 `tools\serial_tester.py`，

```
python serial_tester.py -p COM3 monitor
```

应该每 50ms 收到一帧状态上报（20Hz）。没有输出的话，用调试器看一下是不是停在 HardFault。
