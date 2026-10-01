/**
 * main.c - C1 运动控制主程序（STM32F407ZGT6 + FreeRTOS）
 *
 * 外设分配：
 *   USART3  PB10(TX)/PB11(RX)   与树莓派通信，115200 8N1
 *   TIM3    PB4/PB5/PB0/PB1     两路电机的 RPWM/LPWM，20kHz
 *   TIM2    PA15/PB3            左轮编码器（编码器模式 TI12，32 位）
 *   TIM8    PC6/PC7             右轮编码器（编码器模式 TI12）
 *   TIM7                        1kHz 控制节拍
 *   IWDG                        独立看门狗
 */
#include "main.h"
#include "config.h"
#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"

UART_HandleTypeDef huart3;
DMA_HandleTypeDef  hdma_usart3_tx;
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim3;
TIM_HandleTypeDef  htim7;
TIM_HandleTypeDef  htim8;
IWDG_HandleTypeDef hiwdg;

static uint8_t s_rx_byte;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM7_Init(void);
static void MX_IWDG_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_TIM3_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_TIM7_Init();
    MX_IWDG_Init();

    App_Init();                                     /* 建队列、建任务、初始化电机与编码器 */

    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
    HAL_TIM_Base_Start_IT(&htim7);                  /* 启动 1kHz 节拍 */

    vTaskStartScheduler();

    while (1) { }                                   /* 正常情况下不会到这里 */
}

/* ---------------- 中断回调 ---------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        App_OnRxByte(s_rx_byte);
        HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        App_Tick1kHz();
    }
}

/* ---------------- FreeRTOS 钩子 ---------------- */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}

/* ---------------- 时钟：HSE 8MHz -> 168MHz ---------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;   /* 168MHz */
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;    /* 168MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;     /* 42MHz  */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;     /* 84MHz  */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* ---------------- 外设初始化 ---------------- */

static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = PROTO_BAUD;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

/* 20kHz PWM：TIM3 时钟 84MHz，ARR=4199 */
static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = PWM_ARR;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
}

/* 左轮编码器：TIM2 编码器模式，32 位 */
static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFu;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 0;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) {
        Error_Handler();
    }
}

/* 右轮编码器：TIM8 编码器模式 */
static void MX_TIM8_Init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0xFFFFu;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 0;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim8, &enc) != HAL_OK) {
        Error_Handler();
    }
}

/* 1kHz 控制节拍：TIM7 时钟 84MHz */
static void MX_TIM7_Init(void)
{
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 84 - 1;          /* 1MHz 计数 */
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 1000 - 1;           /* 1kHz */
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
}

/* 看门狗约 1.25s（LSI 32kHz / 64 / 625） */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 625;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

/* 仅初始化 BTS7960 的使能脚（其余引脚在各自 MspInit 里配置） */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*
     * PB4/PB5 既是我们电机 1 的 PWM 输出，也是板上 W25Q128 Flash 的 SPI 引脚。
     * Flash 的片选 PG8 复位后是输入（悬空），可能被误选中，从而让 Flash 的 DO
     * 驱动到 PB4 上造成冲突。这里显式把 PG8 拉高，确保 Flash 始终不被选中。
     */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
    gi.Pin = GPIO_PIN_8;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &gi);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

    gi.Pin = GPIO_PIN_5;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gi);

    gi.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOF, &gi);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}
