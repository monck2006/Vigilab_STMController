/**
 * stm32f4xx_it.c - 中断服务程序
 *
 * 注意：USART3 与 TIM7 的中断里会调用 FreeRTOS 的 ...FromISR 接口，
 *       所以它们的抢占优先级必须 ≥ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)。
 */
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef  htim7;
extern DMA_HandleTypeDef  hdma_usart3_tx;

/* FreeRTOS GCC 端口的滴答处理函数（在 port.c 中定义） */
extern void xPortSysTickHandler(void);

/* ---------------- Cortex-M4 内核异常 ---------------- */

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) { }
}

void MemManage_Handler(void)
{
    while (1) { }
}

void BusFault_Handler(void)
{
    while (1) { }
}

void UsageFault_Handler(void)
{
    while (1) { }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/* ---------------- 外设中断 ---------------- */

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim7);
}

void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}
