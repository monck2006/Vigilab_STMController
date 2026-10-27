/**
 * app_tasks.h - 应用层与中断的接口
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include <stdint.h>

/** 初始化驱动/编码器并创建队列和 FreeRTOS 任务。 */
void App_Init(void);

/** 由 Control 任务在调度器启动后调用，启动 USART3 的首次接收中断。 */
void App_StartUartRx(void);

/** 串口每收到一个字节就调用（中断上下文），内部走协议解析状态机 */
void App_OnRxByte(uint8_t byte);

/** 1kHz TIM7 中断里调用，通知 Control 任务。 */
void App_Tick1kHz(void);

#endif /* __APP_TASKS_H */
