# C1 外设配置参考

当前仓库直接在 `main.c` 和 `stm32f4xx_hal_msp.c` 初始化外设；本文件用于重建工程或核对板级设置。不要按旧版 DMA/多任务配置照抄。

## FreeRTOS 与中断

| 项目 | 当前需要 |
| --- | --- |
| `configTICK_RATE_HZ` | 1000 Hz |
| 抢占调度 | 开启 |
| `configMAX_PRIORITIES` | 至少 6（控制任务优先级 5，Telemetry 2） |
| `configCHECK_FOR_STACK_OVERFLOW` | 建议设为 2 |
| USART3 中断优先级 | 5，允许调用 `xQueueSendFromISR` |
| TIM7 中断优先级 | 6，允许调用 FreeRTOS 任务通知 FromISR |

FreeRTOS API 可调用的中断优先级数值需不小于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（当前为 5）。

## 外设

| 外设 | 配置 |
| --- | --- |
| RCC | HSE 8 MHz，SYSCLK 168 MHz；APB1 42 MHz，APB2 84 MHz |
| USART3 | PB10 TX、PB11 RX；115200、8N1；逐字节接收中断；发送由 Telemetry 任务轮询完成，不启用 TX DMA |
| TIM3 | PWM CH1=PB4、CH2=PB5、CH3=PB0、CH4=PB1；84 MHz 计时输入、ARR=4199、PWM 约 20 kHz |
| TIM2 | TI12 编码器模式，CH1=PA15、CH2=PB3；32 位计数 |
| TIM8 | TI12 编码器模式，CH1=PC6、CH2=PC7；16 位计数 |
| TIM7 | 1 kHz 更新中断；调度器启动后由 Control 任务开始运行 |
| IWDG | 当前预分频 64、重装值 625；控制任务每 500 ms 左右刷新 |
| SYS Debug | Serial Wire / SWD；不要使用完整 JTAG |

## 引脚复用检查

- PB4/PB5 同时连板载 W25Q128，PB0/PB1 与触摸屏复用。使用这些脚输出 PWM 时，相关板载功能不得同时启用；固件把 Flash 片选 PG8 拉高。
- PA15/PB3 兼任 JTAG，选择 SWD 调试。
- PC6/PC7 同时是 USART6，使用 TIM8 编码器时不能再用 USART6。
- PA5/PF11 与触摸屏复用，默认不配置为 BTS7960 EN；驱动板 EN 按实际模块接线拉高。
- PA0 是 UART4_TX 引出，默认不控制电机电源开关。

如果重建工程，先确保 `C1_Keil/Core` 下应用源码只加入一次，并保留与 Keil/GCC 一致的 HAL、CMSIS 和 FreeRTOS 配置。
