/** motor.h - BTS7960 PWM 输出抽象。占空比范围是 -1000..1000。 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_NUM
} motor_id_t;

void Motor_Init(void);
void Motor_SetDuty(motor_id_t motor, int16_t duty_permille);
void Motor_SetEnable(bool enable);
void Motor_CoastAll(void);

/* 只有启用软件电源开关配置时才会实际控制 GPIO。 */
void Motor_PowerOn(void);
void Motor_PowerCut(void);
bool Motor_IsPowerOn(void);
bool Motor_IsEnabled(void);
int16_t Motor_GetDuty(motor_id_t motor);

#endif /* __MOTOR_H */
