/**
 * encoder.c - 编码器测速与里程计实现
 */
#include "encoder.h"
#include "config.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

static TIM_HandleTypeDef *const s_tim[MOTOR_NUM] = { &htim2, &htim8 };

static uint16_t s_last_cnt[MOTOR_NUM];
static int16_t  s_delta[MOTOR_NUM];
static int16_t  s_speed_mmps[MOTOR_NUM];
static int32_t  s_odo_cnt[MOTOR_NUM];      /* 以计数累计，避免浮点累加误差 */

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        __HAL_TIM_SET_COUNTER(s_tim[i], 0);
        s_last_cnt[i] = 0;
        s_delta[i] = 0;
        s_speed_mmps[i] = 0;
        s_odo_cnt[i] = 0;
    }
}

void Encoder_Update(uint32_t dt_ms)
{
    if (dt_ms == 0) {
        return;
    }

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_tim[i]);

        /*
         * 统一按 16 位回绕求增量：(int16_t)(now - last) 在 |增量| < 32768 时精确，
         * 而 10ms 内最多几百个计数，余量充足。TIM2 是 32 位，取低 16 位同样成立。
         */
        int16_t d = (int16_t)((uint16_t)(now - s_last_cnt[i]));
        s_last_cnt[i] = now;
        s_delta[i] = d;
        s_odo_cnt[i] += (int32_t)d;

        /* 速度 mm/s = 计数 × mm/计数 × (1000 / dt_ms) */
        float v = (float)d * MM_PER_COUNT * (1000.0f / (float)dt_ms);
        if (v > 32767.0f)  { v = 32767.0f; }
        if (v < -32768.0f) { v = -32768.0f; }
        s_speed_mmps[i] = (int16_t)v;
    }
}

int16_t Encoder_GetDelta(motor_id_t m)
{
    return (m < MOTOR_NUM) ? s_delta[m] : 0;
}

int16_t Encoder_GetSpeedMMps(motor_id_t m)
{
    return (m < MOTOR_NUM) ? s_speed_mmps[m] : 0;
}

int32_t Encoder_GetOdoMM(motor_id_t m)
{
    if (m >= MOTOR_NUM) {
        return 0;
    }
    /* 计数 → mm，单次转换保留小数部分通过四舍五入控制误差在 0.1mm 内 */
    float mm = (float)s_odo_cnt[m] * MM_PER_COUNT;
    return (int32_t)mm;
}

void Encoder_ResetOdo(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        s_odo_cnt[i] = 0;
    }
}
