/**
 * encoder.h - 编码器正交解码与里程计
 *
 * 硬件：TIM2（PA15/PB3，32 位）驱动左轮，TIM8（PC6/PC7）驱动右轮，
 *       均配置为 Encoder Mode TI12（A/B 双相四倍频）。
 * 换算：11 线 × 4 倍频 × 1:56 = 2464 计数 / 轮子一圈，0.1275 mm / 计数。
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "motor.h"

void Encoder_Init(void);

/**
 * 周期调用（建议 100Hz）。读取计数增量、更新速度与累计里程。
 * dt_ms 为距上次调用的毫秒数。
 */
void Encoder_Update(uint32_t dt_ms);

/** 最近一次采样间隔内的计数增量（带符号） */
int16_t Encoder_GetDelta(motor_id_t m);

/** 线速度 mm/s（带符号，正=前进方向） */
int16_t Encoder_GetSpeedMMps(motor_id_t m);

/** 累计里程 mm（带符号） */
int32_t Encoder_GetOdoMM(motor_id_t m);

/** 里程计清零 */
void Encoder_ResetOdo(void);

#endif /* __ENCODER_H */
