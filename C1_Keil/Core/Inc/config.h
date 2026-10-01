/**
 * config.h - C1 运动控制全部可调参数
 *
 * 标定或换电机后，只改这一个文件，其余代码不用动。
 */
#ifndef __CONFIG_H
#define __CONFIG_H

/* ===================== 1. 机械与电机参数 ===================== */

#define ENC_PPR                 11      /* 编码器线数（单相每转脉冲）；若标定为 13 线改成 13 */
#define ENC_MULT                4       /* 正交解码四倍频 */
#define GEAR_RATIO              56.0f   /* 减速比 1:56（JGB37-555-1280-EN） */
#define WHEEL_DIA_MM            180.0f  /* 驱动轮直径 mm（M1 机械设计：主动轮 Φ180） */
#define WHEEL_TRACK_MM          428.0f  /* 两驱动轮中心距 mm（机械设计确定值） */

/* 编码器计数：电机转一圈 44，轮子转一圈 2464 */
#define ENC_CNT_PER_MOTOR_REV   ((int32_t)(ENC_PPR * ENC_MULT))
#define ENC_CNT_PER_WHEEL_REV   ((int32_t)(ENC_CNT_PER_MOTOR_REV * GEAR_RATIO))

/* 轮子每转前进距离 mm = 565.5（Φ180） */
#define WHEEL_CIRC_MM           (3.14159265f * WHEEL_DIA_MM)
/* 每个编码器计数对应的位移 mm = 0.2295 */
#define MM_PER_COUNT            (WHEEL_CIRC_MM / (float)ENC_CNT_PER_WHEEL_REV)

/* ===================== 2. 控制参数 ===================== */

#define CTRL_TICK_HZ            1000            /* 控制节拍 1kHz（TIM7） */
#define SPEED_LOOP_DIV          5               /* 每 5 个节拍跑一次速度环 = 200Hz */
#define PWM_FREQ_HZ             20000           /* PWM 频率 20kHz */
#define PWM_ARR                 4199            /* 84MHz / (4199+1) / 1 = 20kHz */

#define VEL_MAX_MMPS            1000            /* 速度上限 mm/s（额定 1008，空载 1338） */
#define ACC_MAX_MMPS2           500             /* 加速度上限 mm/s^2，用于软启动 */
#define PWM_DEADZONE_PCT        8               /* 占空比死区，低于此值电机不转 */

/* 速度环 PI（增量式），先用经验值，实车再调 */
#define PID_KP                  0.30f
#define PID_KI                  0.05f

/* ===================== 3. 安全参数（受电池 10A 约束）===================== */

#define COMM_TIMEOUT_MS         500     /* 通信超时 → 软停 */
#define COMM_LOST_MS            1000    /* 继续丢失 → 急停锁定 */
#define MOTOR_RATED_A           1.5f    /* 单电机额定电流 A（JGB37-555-1280-EN 参数表） */
#define MOTOR_STALL_A           6.5f    /* 单电机堵转电流 A（同上） */
#define BUS_CURRENT_LIMIT_A     8.0f    /* 母线限流阈值（电池持续 10A，留 2A 余量） */
#define STALL_DUTY_PCT          30      /* 判定堵转的占空比下限 */
#define STALL_TIME_MS           500     /* 堵转判定时间 */
#define VBAT_UV_WARN_MV         10500   /* 欠压告警 10.5V */
#define VBAT_UV_STOP_MV         10000   /* 欠压停机 10.0V */
#define VBAT_DIVIDER            11.0f   /* 分压比，按实际采样电阻调整 */

/* ===================== 4. 通信参数 ===================== */

#define PROTO_BAUD              115200
#define STATUS_REPORT_HZ        20      /* 状态帧上报频率 */
#define FRAME_GAP_TIMEOUT_MS    10      /* 帧内字节间隔超时，超时丢弃该帧 */

/* ===================== 5. 调试 ===================== */

#define DBG_UART                1       /* 1=打开调试串口打印 */

#endif /* __CONFIG_H */
