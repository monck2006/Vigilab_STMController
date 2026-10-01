/**
 * stm32f4xx_hal_msp.c - 外设底层初始化（引脚、时钟、DMA、中断）
 */
#include "main.h"

extern DMA_HandleTypeDef hdma_usart3_tx;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* ---------------- USART3: PB10=TX, PB11=RX ---------------- */

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gi = {0};

    if (huart->Instance != USART3) {
        return;
    }

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gi.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gi);

    /* TX 用 DMA，减少任务阻塞 */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    HAL_DMA_DeInit(huart->hdmatx);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
}

/* ---------------- TIM3: 两路电机 PWM ---------------- */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gi = {0};

    if (htim->Instance != TIM3) {
        return;
    }

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB4=CH1, PB5=CH2, PB0=CH3, PB1=CH4，都是 AF2 */
    gi.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_0 | GPIO_PIN_1;
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gi);
}

/* ---------------- TIM2 / TIM8: 编码器 ---------------- */

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gi = {0};

    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PA15=CH1(AF1), PB3=CH2(AF1) */
        gi.Pin = GPIO_PIN_15;
        gi.Mode = GPIO_MODE_AF_PP;
        gi.Pull = GPIO_PULLUP;
        gi.Speed = GPIO_SPEED_FREQ_HIGH;
        gi.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &gi);

        gi.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOB, &gi);
    } else if (htim->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* PC6=CH1(AF3), PC7=CH2(AF3) */
        gi.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gi.Mode = GPIO_MODE_AF_PP;
        gi.Pull = GPIO_PULLUP;
        gi.Speed = GPIO_SPEED_FREQ_HIGH;
        gi.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &gi);
    }
}

/* ---------------- TIM7: 1kHz 节拍 ---------------- */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7) {
        return;
    }
    __HAL_RCC_TIM7_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM7_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}
