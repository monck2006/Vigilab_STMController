/**
 * encoder.h - 读取左右轮 AB 正交编码器，计算速度和累计里程。
 *
 * 硬件：TIM2 左轮、TIM8 右轮。每次 Encoder_Update() 应传入真实经过的毫秒数。
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "motor.h"

void Encoder_Init(void);
void Encoder_Update(uint32_t dt_ms);

int16_t Encoder_GetDelta(motor_id_t motor);
int16_t Encoder_GetSpeedMMps(motor_id_t motor);
int32_t Encoder_GetOdoMM(motor_id_t motor);
void Encoder_ResetOdo(void);

#endif /* __ENCODER_H */
