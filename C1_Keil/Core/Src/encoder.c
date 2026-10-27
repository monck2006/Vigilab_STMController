/**
 * encoder.c - 编码器计数 -> 速度/里程。
 *
 * TIM2 是 32 位、TIM8 是 16 位。这里统一使用低 16 位的回绕差值；
 * 前提是两次采样之间的计数变化小于 32768，20 ms 采样足够满足该条件。
 */
#include "encoder.h"
#include "config.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

static TIM_HandleTypeDef *const s_timer[MOTOR_NUM] = { &htim2, &htim8 };
static const int8_t s_encoder_sign[MOTOR_NUM] = {
    ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN
};

static uint16_t s_last_count[MOTOR_NUM];
static int16_t s_delta[MOTOR_NUM];
static int16_t s_speed_mmps[MOTOR_NUM];
static int32_t s_total_count[MOTOR_NUM];

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        __HAL_TIM_SET_COUNTER(s_timer[i], 0u);
        s_last_count[i] = 0u;
        s_delta[i] = 0;
        s_speed_mmps[i] = 0;
        s_total_count[i] = 0;
    }
}

void Encoder_Update(uint32_t dt_ms)
{
    if (dt_ms == 0u || ENCODER_COUNTS_PER_WHEEL_REV == 0u) {
        return;
    }

    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_timer[i]);
        int16_t raw_delta = (int16_t)(uint16_t)(now - s_last_count[i]);
        int32_t delta = (int32_t)raw_delta * (int32_t)s_encoder_sign[i];
        float wheel_mm_per_count =
            (3.14159265f * WHEEL_DIAMETER_MM) /
            (float)ENCODER_COUNTS_PER_WHEEL_REV;
        float speed = (float)delta * wheel_mm_per_count *
                      (1000.0f / (float)dt_ms);

        s_last_count[i] = now;
        s_delta[i] = (int16_t)delta;
        s_total_count[i] += delta;

        if (speed > 32767.0f) { speed = 32767.0f; }
        if (speed < -32768.0f) { speed = -32768.0f; }
        s_speed_mmps[i] = (int16_t)speed;
    }
}

int16_t Encoder_GetDelta(motor_id_t motor)
{
    return (motor < MOTOR_NUM) ? s_delta[motor] : 0;
}

int16_t Encoder_GetSpeedMMps(motor_id_t motor)
{
    return (motor < MOTOR_NUM) ? s_speed_mmps[motor] : 0;
}

int32_t Encoder_GetOdoMM(motor_id_t motor)
{
    if (motor >= MOTOR_NUM) {
        return 0;
    }

    float mm = (float)s_total_count[motor] *
               (3.14159265f * WHEEL_DIAMETER_MM) /
               (float)ENCODER_COUNTS_PER_WHEEL_REV;
    return (int32_t)((mm >= 0.0f) ? (mm + 0.5f) : (mm - 0.5f));
}

void Encoder_ResetOdo(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        s_total_count[i] = 0;
    }
}
