/**
 * motor.c - BTS7960 双 PWM 输出。
 *
 * 正占空比使用 RPWM，负占空比使用 LPWM；实际左右电机方向可在 config.h 反转。
 * 默认不占用开发板的 EN 或 PA0 引脚：EN 应按驱动模块接线说明固定为高电平。
 */
#include "motor.h"
#include "config.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

static const uint32_t s_rpwm_channel[MOTOR_NUM] = {
    TIM_CHANNEL_1, TIM_CHANNEL_3
};
static const uint32_t s_lpwm_channel[MOTOR_NUM] = {
    TIM_CHANNEL_2, TIM_CHANNEL_4
};
static const int8_t s_motor_sign[MOTOR_NUM] = {
    MOTOR_LEFT_SIGN, MOTOR_RIGHT_SIGN
};

static int16_t s_duty[MOTOR_NUM];
static bool s_enabled;

#if MOTOR_USE_EN_PINS
#define EN_LEFT_PORT    GPIOA
#define EN_LEFT_PIN     GPIO_PIN_5
#define EN_RIGHT_PORT   GPIOF
#define EN_RIGHT_PIN    GPIO_PIN_11
#endif

#if MOTOR_USE_POWER_SWITCH
/* PA0 is UART4_TX on the development board. Enable only after external wiring is verified. */
#define POWER_PORT      GPIOA
#define POWER_PIN       GPIO_PIN_0
#if MOTOR_POWER_SWITCH_ACTIVE_HIGH
#define POWER_ON_LEVEL  GPIO_PIN_SET
#define POWER_OFF_LEVEL GPIO_PIN_RESET
#else
#define POWER_ON_LEVEL  GPIO_PIN_RESET
#define POWER_OFF_LEVEL GPIO_PIN_SET
#endif
static bool s_power_on;
#endif

static void set_compare(uint32_t channel, uint16_t permille)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t compare;

    if (permille > PWM_MAX_PERMILLE) {
        permille = PWM_MAX_PERMILLE;
    }
    compare = (uint32_t)(((uint64_t)(arr + 1u) * permille) / 1000u);
    if (compare > arr + 1u) {
        compare = arr + 1u;
    }
    __HAL_TIM_SET_COMPARE(&htim3, channel, compare);
}

static void set_motor_pwm(motor_id_t motor, int16_t physical_duty)
{
    uint16_t magnitude = (uint16_t)((physical_duty < 0) ?
                                    -physical_duty : physical_duty);

    if (physical_duty > 0) {
        set_compare(s_rpwm_channel[motor], magnitude);
        set_compare(s_lpwm_channel[motor], 0u);
    } else if (physical_duty < 0) {
        set_compare(s_rpwm_channel[motor], 0u);
        set_compare(s_lpwm_channel[motor], magnitude);
    } else {
        set_compare(s_rpwm_channel[motor], 0u);
        set_compare(s_lpwm_channel[motor], 0u);
    }
}

void Motor_Init(void)
{
#if MOTOR_USE_EN_PINS
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = EN_LEFT_PIN;
    HAL_GPIO_Init(EN_LEFT_PORT, &gpio);
    gpio.Pin = EN_RIGHT_PIN;
    HAL_GPIO_Init(EN_RIGHT_PORT, &gpio);
    HAL_GPIO_WritePin(EN_LEFT_PORT, EN_LEFT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_RIGHT_PORT, EN_RIGHT_PIN, GPIO_PIN_RESET);
#endif

#if MOTOR_USE_POWER_SWITCH
    GPIO_InitTypeDef power_gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(POWER_PORT, POWER_PIN, POWER_OFF_LEVEL);
    power_gpio.Pin = POWER_PIN;
    power_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    power_gpio.Pull = GPIO_NOPULL;
    power_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(POWER_PORT, &power_gpio);
    s_power_on = false;
#endif

    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        s_duty[i] = 0;
        set_motor_pwm((motor_id_t)i, 0);
    }

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    s_enabled = false;
}

void Motor_SetDuty(motor_id_t motor, int16_t duty_permille)
{
    int32_t logical_duty;
    int32_t physical_duty;
    int32_t deadzone = PWM_DEADZONE_PERMILLE;

    if (motor >= MOTOR_NUM) {
        return;
    }
    if (!s_enabled) {
        set_motor_pwm(motor, 0);
        s_duty[motor] = 0;
        return;
    }

    logical_duty = duty_permille;
    if (logical_duty > PWM_MAX_PERMILLE) { logical_duty = PWM_MAX_PERMILLE; }
    if (logical_duty < -PWM_MAX_PERMILLE) { logical_duty = -PWM_MAX_PERMILLE; }

    if (logical_duty > 0 && logical_duty < deadzone) { logical_duty = 0; }
    if (logical_duty < 0 && logical_duty > -deadzone) { logical_duty = 0; }

    physical_duty = logical_duty * (int32_t)s_motor_sign[motor];
    set_motor_pwm(motor, (int16_t)physical_duty);
    s_duty[motor] = (int16_t)logical_duty;
}

void Motor_CoastAll(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        set_motor_pwm((motor_id_t)i, 0);
        s_duty[i] = 0;
    }
}

void Motor_SetEnable(bool enable)
{
    s_enabled = enable;
#if MOTOR_USE_EN_PINS
    HAL_GPIO_WritePin(EN_LEFT_PORT, EN_LEFT_PIN,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_RIGHT_PORT, EN_RIGHT_PIN,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    if (!enable) {
        Motor_CoastAll();
    }
}

void Motor_PowerOn(void)
{
#if MOTOR_USE_POWER_SWITCH
    HAL_GPIO_WritePin(POWER_PORT, POWER_PIN, POWER_ON_LEVEL);
    s_power_on = true;
#endif
}

void Motor_PowerCut(void)
{
#if MOTOR_USE_POWER_SWITCH
    HAL_GPIO_WritePin(POWER_PORT, POWER_PIN, POWER_OFF_LEVEL);
    s_power_on = false;
#endif
}

bool Motor_IsPowerOn(void)
{
#if MOTOR_USE_POWER_SWITCH
    return s_power_on;
#else
    return false; /* 没有反馈信号时不假报“电源已接通”。 */
#endif
}

bool Motor_IsEnabled(void)
{
    return s_enabled;
}

int16_t Motor_GetDuty(motor_id_t motor)
{
    return (motor < MOTOR_NUM) ? s_duty[motor] : 0;
}
