/**
 * motor.c - BTS7960 H 桥驱动实现
 *
 * BTS7960 真值表：
 *   RPWM=PWM, LPWM=0  -> 正转调速
 *   RPWM=0,   LPWM=PWM-> 反转调速
 *   RPWM=0,   LPWM=0  -> 自由停车
 *   RPWM=1,   LPWM=1  -> 刹车
 */
#include "motor.h"
#include "config.h"
#include "stm32f4xx_hal.h"

/* CubeMX 生成的句柄，这里只做声明，避免依赖 tim.h */
extern TIM_HandleTypeDef htim3;

#if MOTOR_USE_EN
#define EN1_PORT    GPIOA
#define EN1_PIN     GPIO_PIN_5      /* PA5  */
#define EN2_PORT    GPIOF
#define EN2_PIN     GPIO_PIN_11     /* PF11 */
#endif

#if MOTOR_USE_PWR_SWITCH
#define PWR_PORT    GPIOA
#define PWR_PIN     GPIO_PIN_0      /* PA0  */
#if PWR_SWITCH_HIGH_IS_ON
#define PWR_ON_LEVEL    GPIO_PIN_SET
#define PWR_OFF_LEVEL   GPIO_PIN_RESET
#else
#define PWR_ON_LEVEL    GPIO_PIN_RESET
#define PWR_OFF_LEVEL   GPIO_PIN_SET
#endif
static bool s_power_on;
#endif

/* 每个电机的两路比较通道 */
static const uint32_t s_ch_rpwm[MOTOR_NUM] = { TIM_CHANNEL_1, TIM_CHANNEL_3 };
static const uint32_t s_ch_lpwm[MOTOR_NUM] = { TIM_CHANNEL_2, TIM_CHANNEL_4 };

static int16_t s_duty[MOTOR_NUM];
static bool    s_enabled;

/* 占空比千分比 -> CCR */
static void set_pwm(uint32_t channel, uint16_t permille)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr;

    if (permille > 1000u) {
        permille = 1000u;
    }
    ccr = (uint32_t)(((uint64_t)(arr + 1u) * permille) / 1000u);
    if (ccr > arr + 1u) {
        ccr = arr + 1u;
    }
    __HAL_TIM_SET_COMPARE(&htim3, channel, ccr);
}

#if MOTOR_USE_EN
static void enable_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pin = EN1_PIN;
    HAL_GPIO_Init(EN1_PORT, &gi);
    gi.Pin = EN2_PIN;
    HAL_GPIO_Init(EN2_PORT, &gi);
}

static void power_switch_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 先输出"断开"电平，再配置成输出，保证上电瞬间是断电状态 */
    HAL_GPIO_WritePin(PWR_PORT, PWR_PIN, PWR_OFF_LEVEL);
    gi.Pin = PWR_PIN;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PWR_PORT, &gi);
    s_power_on = false;
}
#endif

void Motor_Init(void)
{
    /* 先把两路都置 0，避免上电瞬间电机抖动 */
    set_pwm(s_ch_rpwm[MOTOR_LEFT],  0u);
    set_pwm(s_ch_lpwm[MOTOR_LEFT],  0u);
    set_pwm(s_ch_rpwm[MOTOR_RIGHT], 0u);
    set_pwm(s_ch_lpwm[MOTOR_RIGHT], 0u);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

#if MOTOR_USE_EN
    enable_gpio_init();
    HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, GPIO_PIN_RESET);
#endif

#if MOTOR_USE_PWR_SWITCH
    power_switch_gpio_init();
#endif

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        s_duty[i] = 0;
    }
    s_enabled = false;
}

#if MOTOR_USE_PWR_SWITCH
void Motor_PowerOn(void)
{
    HAL_GPIO_WritePin(PWR_PORT, PWR_PIN, PWR_ON_LEVEL);
    s_power_on = true;
}

void Motor_PowerCut(void)
{
    HAL_GPIO_WritePin(PWR_PORT, PWR_PIN, PWR_OFF_LEVEL);
    s_power_on = false;
}

bool Motor_IsPowerOn(void)
{
    return s_power_on;
}
#else
void Motor_PowerOn(void)  { }
void Motor_PowerCut(void) { }
bool Motor_IsPowerOn(void){ return true; }
#endif

void Motor_SetEnable(bool en)
{
    s_enabled = en;
#if MOTOR_USE_EN
    HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)en;
#endif
    if (!en) {
        Motor_CoastAll();
    }
}

void Motor_SetDuty(motor_id_t m, int16_t duty)
{
    uint16_t mag;

    if (m >= MOTOR_NUM) {
        return;
    }
    if (!s_enabled) {
        Motor_Coast(m);
        s_duty[m] = 0;
        return;
    }

    if (duty > 1000)  { duty = 1000; }
    if (duty < -1000) { duty = -1000; }

    /* 死区：低于阈值直接停，避免电机堵转发热 */
    if (duty > 0 && duty < PWM_DEADZONE_PCT * 10) {
        duty = 0;
    } else if (duty < 0 && duty > -PWM_DEADZONE_PCT * 10) {
        duty = 0;
    }

    mag = (uint16_t)((duty < 0) ? -duty : duty);

    if (duty > 0) {
        set_pwm(s_ch_rpwm[m], mag);
        set_pwm(s_ch_lpwm[m], 0u);
    } else if (duty < 0) {
        set_pwm(s_ch_rpwm[m], 0u);
        set_pwm(s_ch_lpwm[m], mag);
    } else {
        set_pwm(s_ch_rpwm[m], 0u);
        set_pwm(s_ch_lpwm[m], 0u);
    }
    s_duty[m] = duty;
}

void Motor_Brake(motor_id_t m)
{
    if (m >= MOTOR_NUM) {
        return;
    }
    set_pwm(s_ch_rpwm[m], 1000u);
    set_pwm(s_ch_lpwm[m], 1000u);
    s_duty[m] = 0;
}

void Motor_Coast(motor_id_t m)
{
    if (m >= MOTOR_NUM) {
        return;
    }
    set_pwm(s_ch_rpwm[m], 0u);
    set_pwm(s_ch_lpwm[m], 0u);
    s_duty[m] = 0;
}

void Motor_BrakeAll(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        Motor_Brake((motor_id_t)i);
    }
}

void Motor_CoastAll(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        Motor_Coast((motor_id_t)i);
    }
}

int16_t Motor_GetDuty(motor_id_t m)
{
    return (m < MOTOR_NUM) ? s_duty[m] : 0;
}
