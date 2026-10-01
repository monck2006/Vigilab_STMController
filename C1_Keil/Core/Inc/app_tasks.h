/**
 * app_tasks.h - 应用层与中断的接口
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include <stdint.h>

/** 创建队列与 FreeRTOS 任务，在 main() 里 osKernelStart() 之前调用 */
void App_Init(void);

/** 串口每收到一个字节就调用（中断上下文），内部走协议解析状态机 */
void App_OnRxByte(uint8_t byte);

/** 1kHz 定时器中断里调用（TIM7），作为控制节拍 */
void App_Tick1kHz(void);

#endif /* __APP_TASKS_H */
