/**
 * motor.h - BTS7960 H 桥驱动层
 *
 * 引脚（见 README 的引脚分配表）：
 *   电机1  RPWM=PB4(TIM3_CH1)  LPWM=PB5(TIM3_CH2)
 *   电机2  RPWM=PB0(TIM3_CH3)  LPWM=PB1(TIM3_CH4)
 *   EN1=PA5  EN2=PF11（可选，不接时把 MOTOR_USE_EN 设为 0）
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_USE_EN        1       /* 1=使用 EN 引脚控制使能；0=EN 直接接 3.3V */

/*
 * 急停采用「断电」方案：由 STM32 控制底板那路可接入控制板的电源开关。
 * 断开时电机驱动板的 12V 被完全切掉，即使 MCU 跑飞也不会转。
 *   PA0 = 电源开关控制（P7 引出接口第 8 脚）
 * 上电默认断电，收到运动指令后才合闸；急停 / 通信超时立即断闸。
 */
#define MOTOR_USE_PWR_SWITCH    1       /* 0=不用 MCU 控制电源开关（改由物理按钮断电） */
#define PWR_SWITCH_HIGH_IS_ON   1       /* 1=输出高电平合闸；若底板上是低电平有效改成 0 */

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_NUM
} motor_id_t;

/** 初始化 TIM3 四路 PWM 与 EN 引脚（调用前 CubeMX 已配置 TIM3 为 PWM 模式） */
void Motor_Init(void);

/**
 * 设置某个电机的输出。
 * duty: -1000 ~ +1000 千分比，正=正转，负=反转，0=自由停车
 * 内部自动做死区处理与限幅。
 */
void Motor_SetDuty(motor_id_t m, int16_t duty);

/** 刹车：RPWM=LPWM=1（能耗制动），用于急停 */
void Motor_Brake(motor_id_t m);

/** 自由停车：RPWM=LPWM=0 */
void Motor_Coast(motor_id_t m);

/** 两个电机一起刹车 / 自由停车 */
void Motor_BrakeAll(void);
void Motor_CoastAll(void);

/** 使能/禁用输出级（拉低 EN 时电机断电） */
void Motor_SetEnable(bool en);

/** 驱动电源开关：断开/合闸（急停用断电时由它执行） */
void Motor_PowerOn(void);
void Motor_PowerCut(void);
bool Motor_IsPowerOn(void);

/** 读取当前设定的占空比千分比（带符号） */
int16_t Motor_GetDuty(motor_id_t m);

#endif /* __MOTOR_H */
