/**
 * config.h - 首次上车前只需要重点检查这里的硬件参数。
 *
 * 注意：编码器资料中的“16 PPR”没有说明是电机轴还是减速箱输出轴。
 * 这里先按电机轴 16 PPR、A/B 四倍频、减速比 56:1 配置；请按实测校准。
 */
#ifndef __CONFIG_H
#define __CONFIG_H

/* 机械参数 */
#define WHEEL_DIAMETER_MM              180.0f
#define WHEEL_TRACK_MM                 428.0f

/* 初始假设：16 脉冲/电机轴转 × 4 倍频 × 56 减速比 = 3584 计数/轮转 */
#define ENCODER_COUNTS_PER_WHEEL_REV   3584u
/* 左右轮方向归一化：正数保持，-1 表示反向。抬轮调试时两侧前进都应读正数。 */
#define ENCODER_LEFT_SIGN              1
#define ENCODER_RIGHT_SIGN             1
/* 轮子正方向不一致时，在这里单独翻转电机输出方向。 */
#define MOTOR_LEFT_SIGN                1
#define MOTOR_RIGHT_SIGN               1

/* 控制周期与输出 */
#define CONTROL_PERIOD_MS              20u       /* 50 Hz 速度闭环 */
#define VEL_MAX_MMPS                   1000      /* 上位机目标限幅 */
#define ACC_MAX_MMPS2                  300.0f    /* 目标速度斜坡，mm/s^2 */
#define MOTOR_PWM_ARR                  4199u     /* TIM3=84MHz，ARR=4199 时 PWM=20kHz */
#define PWM_MAX_PERMILLE               1000      /* 1000 = 100% */
#define PWM_DEADZONE_PERMILLE          0         /* 测出电机死区后再填写 */

/* 速度 PI：输出单位为 PWM 千分比，误差单位为 mm/s。
 * I[k] = I[k-1] + Ki * error * dt。参数是起始值，必须实车低速调试。 */
#define SPEED_PI_KP                    0.30f
#define SPEED_PI_KI_PER_SECOND         2.0f
#define SPEED_PI_INTEGRAL_LIMIT        400.0f

/* 停车和通信保护 */
#define STOP_SPEED_MMPS                20
#define COMM_TIMEOUT_MS                500u
#define COMM_LOST_MS                   1000u
#define STATUS_REPORT_PERIOD_MS        50u       /* 20 Hz */
#define UART_TX_TIMEOUT_MS              20u

/* 当前没有电流/电池 ADC 采样实现，不启用估算式过流和欠压保护。
 * EN 引脚可能与开发板触摸屏复用，默认由驱动板端将 R_EN/L_EN 拉高。
 * PA0 是开发板 UART4_TX，不能默认当作电源开关。 */
#define MOTOR_USE_EN_PINS              0
#define MOTOR_USE_POWER_SWITCH         0
#define MOTOR_POWER_SWITCH_ACTIVE_HIGH 1

/* 编码器计数尚未实测前不启用堵转锁定，避免测速参数错误造成误停。 */
#define STALL_DETECT_ENABLE            0
#define STALL_DUTY_PERMILLE            300
#define STALL_TIME_MS                  500u

#define PROTO_BAUD                     115200u

#endif /* __CONFIG_H */
