# C1 固件（STM32F407ZGT6 + FreeRTOS + HAL）

本目录存放 C1 的应用层代码。开发方式：用 STM32CubeMX 生成基础工程（时钟、USART3、TIM3、TIM2、TIM8、IWDG、FreeRTOS），再把这里的文件加进去。

## 目录

```
firmware/
  Core/Inc/config.h        全部可调参数（机械、控制、安全、协议），标定后只改这一个文件
  Core/Inc/protocol.h      上下位机协议：帧结构、命令码、状态码
  Core/Src/protocol.c      协议实现：CRC16-Modbus、逐字节状态机解析、打包
  Core/Inc/motor.h         BTS7960 驱动层接口
  Core/Src/motor.c         PWM 输出、正反转、刹车、自由停车、EN 控制
  Core/Inc/encoder.h       编码器与里程计接口
  Core/Src/encoder.c       TIM2/TIM8 编码器模式读数、测速、里程累计
  Core/Inc/app_tasks.h     中断与任务的接口
  Core/Src/app_tasks.c     FreeRTOS 五个任务、命令处理、状态上报、安全监控
```

## 编译验证

四个 .c 文件已用 Keil MDK 自带的 armclang（`--target=arm-arm-none-eabi -mcpu=cortex-m4 -mfpu=fpv4-sp-d16`）
配 ST 官方 HAL 头文件编译通过，`-Wall -Wextra` 零警告。编译时需定义 `STM32F407xx` 与 `USE_HAL_DRIVER`。

协议层另外做了逻辑自检：正常帧解析正确、单字节翻转能检出 CRC 错误、帧前垃圾字节能自动重同步。

## CubeMX 里 FreeRTOS 的建议配置

| 配置项 | 建议值 |
| --- | --- |
| `configTICK_RATE_HZ` | 1000 |
| `configMAX_PRIORITIES` | 8（任务用到 1～5） |
| `configTOTAL_HEAP_SIZE` | 30 KB（FreeRTOSConfig.h 里改） |
| `configUSE_PREEMPTION` | 1（抢占式） |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 |
| `configMINIMAL_STACK_SIZE` | 128 |
| 中断优先级 | USART3=5、TIM7=6，且都 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`(5) |

## CubeMX 需要配置的外设

| 外设 | 配置 |
| --- | --- |
| RCC | HSE 8MHz → PLL → SYSCLK 168MHz |
| USART3 | 115200 8N1，TX=PB10、RX=PB11；开 DMA（RX 循环、TX 普通）+ 空闲中断 |
| TIM3 | PWM 模式，CH1=PB4、CH2=PB5、CH3=PB0、CH4=PB1；20kHz，ARR=4199（84MHz 时钟） |
| TIM2 | Encoder Mode TI12，CH1=PA15、CH2=PB3；32 位自动重装 |
| TIM8 | Encoder Mode TI12，CH1=PC6、CH2=PC7 |
| TIM7 | 1kHz 定时中断，作为控制节拍 |
| IWDG | 1s 超时 |
| SYS | 调试选 **Serial Wire**（不要选 JTAG，PA15/PB3/PB4 被占用） |

## 上电顺序

1. 先接 12V 电池 → 底板上电 → 测 5V / 3.3V 正常；
2. 再接 STM32 开发板（杜邦线供电或 USB）；
3. 最后接电机（避免上电瞬间抖动导致误转）；
4. 首次调试建议先不接 BTS7960 的电源，只看串口通信与编码器读数。
