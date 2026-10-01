/**
 * app_tasks.c - FreeRTOS 任务：运动控制、通信、安全监控
 *
 * 任务与优先级（详见 docs/01_固件架构与任务划分.md）：
 *   Task_Safety  5  1kHz   安全监控、急停状态机
 *   Task_Motor   4  200Hz  速度环、PWM 输出
 *   Task_Encoder 3  100Hz  编码器采样、里程计
 *   Task_Comm    3  事件   帧解析、命令执行、20Hz 状态上报
 *   Task_Monitor 1  1Hz    看门狗、故障记录
 */
#include "app_tasks.h"
#include "config.h"
#include "protocol.h"
#include "motor.h"
#include "encoder.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* CubeMX 生成的句柄（只做声明，避免依赖 uart.h / iwdg.h） */
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef  hadc1;

/* ---------------- 任务参数 ---------------- */
#define STK_SAFETY      192
#define STK_MOTOR       256
#define STK_ENCODER     192
#define STK_COMM        320
#define STK_MONITOR     160

#define PRIO_SAFETY     5
#define PRIO_MOTOR      4
#define PRIO_ENCODER    3
#define PRIO_COMM       3
#define PRIO_MONITOR    1

/* ---------------- 全局状态 ---------------- */
static QueueHandle_t   s_q_rx;
static TaskHandle_t    s_h_safety;
static TaskHandle_t    s_h_motor;
static proto_parser_t  s_parser;
static proto_frame_t   s_rx_frame;              /* 仅在 USART 中断里写 */

static volatile uint32_t s_tick_ms;             /* 1kHz 计数 */
static volatile uint32_t s_last_rx_ms;          /* 最后一次收到合法帧的时刻 */
static volatile bool     s_ever_rx;             /* 是否收到过第一帧 */
static volatile uint8_t  s_state = ST_INIT;
static volatile uint8_t  s_fault = FAULT_NONE;

static volatile int16_t  s_target[MOTOR_NUM];   /* 目标线速度 mm/s */
static volatile int16_t  s_cmd_duty[MOTOR_NUM]; /* 控制环输出（千分比） */
static volatile bool     s_estop_latch;
static volatile uint32_t s_stall_ms;

static uint8_t  s_tx_seq;
static uint8_t  s_tx_buf[48];                   /* DMA 发送缓冲，必须常驻 */
static uint32_t s_last_report_ms;

/* ---------------- 中断上下文接口 ---------------- */

void App_OnRxByte(uint8_t byte)
{
    BaseType_t hpw = pdFALSE;

    if (proto_parser_feed(&s_parser, byte, &s_rx_frame)) {
        s_last_rx_ms = s_tick_ms;
        s_ever_rx = true;
        (void)xQueueSendFromISR(s_q_rx, (void *)&s_rx_frame, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

void App_Tick1kHz(void)
{
    static uint8_t div = 0;
    BaseType_t hpw = pdFALSE;

    s_tick_ms++;

    if (s_h_safety != NULL) {
        vTaskNotifyGiveFromISR(s_h_safety, &hpw);
    }
    if (++div >= SPEED_LOOP_DIV) {
        div = 0;
        if (s_h_motor != NULL) {
            vTaskNotifyGiveFromISR(s_h_motor, &hpw);
        }
    }
    portYIELD_FROM_ISR(hpw);
}

/* ---------------- 内部工具 ---------------- */

static void fault_set(uint8_t code)
{
    if (s_fault == FAULT_NONE) {
        s_fault = code;
    }
}

/** 取绝对值（避免引入 stdlib） */
static int16_t abs_i16(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

static void emergency_stop(void)
{
    Motor_BrakeAll();
#if MOTOR_USE_EN
    Motor_SetEnable(false);
#endif
    /* 急停采用断电方案：直接切掉驱动板 12V，即使 MCU 跑飞也不会转 */
    Motor_PowerCut();
    s_target[MOTOR_LEFT] = 0;
    s_target[MOTOR_RIGHT] = 0;
    s_cmd_duty[MOTOR_LEFT] = 0;
    s_cmd_duty[MOTOR_RIGHT] = 0;
    s_state = ST_FAULT;
}

static void soft_stop(void)
{
    s_target[MOTOR_LEFT] = 0;
    s_target[MOTOR_RIGHT] = 0;
}

/** 用占空比粗略估算母线电流，用于限流保护 */
static float estimate_bus_current_a(void)
{
    float sum = 0.0f;

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        float duty = (float)(s_cmd_duty[i] < 0 ? -s_cmd_duty[i] : s_cmd_duty[i]) / 1000.0f;
        sum += duty * MOTOR_RATED_A;
    }
    return sum;
}

static void send_status(void)
{
    uint8_t d[17];
    uint16_t n;
    uint16_t vbus_mv = 12300;   /* TODO: 接入 ADC 后改为真实采样值 */

    d[0] = s_state;
    d[1] = s_fault;
    d[2] = (uint8_t)(vbus_mv & 0xFF);
    d[3] = (uint8_t)(vbus_mv >> 8);
    proto_put_i16(&d[4], Encoder_GetSpeedMMps(MOTOR_LEFT));
    proto_put_i16(&d[6], Encoder_GetSpeedMMps(MOTOR_RIGHT));
    proto_put_i32(&d[8], Encoder_GetOdoMM(MOTOR_LEFT));
    proto_put_i32(&d[12], Encoder_GetOdoMM(MOTOR_RIGHT));
    /* flags：bit0=驱动电源已合闸，bit1=输出级使能，bit2=急停锁定 */
    d[16] = (uint8_t)((Motor_IsPowerOn() ? 0x01 : 0x00) |
                      0x02 |
                      (s_estop_latch ? 0x04 : 0x00));

    n = proto_pack(s_tx_buf, sizeof(s_tx_buf), s_tx_seq++, CMD_STATUS, d, sizeof(d));
    if (n > 0) {
        HAL_UART_Transmit_DMA(&huart3, s_tx_buf, n);
    }
}

static void send_ack(uint8_t ack_cmd, uint8_t code)
{
    uint8_t d[2];
    uint16_t n;

    d[0] = ack_cmd;
    d[1] = code;
    n = proto_pack(s_tx_buf, sizeof(s_tx_buf), s_tx_seq++, CMD_ACK, d, sizeof(d));
    if (n > 0) {
        HAL_UART_Transmit_DMA(&huart3, s_tx_buf, n);
    }
}

/* ---------------- 任务实现 ---------------- */

/** 安全监控：1kHz，最高优先级，任何异常立即动作 */
static void Task_Safety(void *arg)
{
    (void)arg;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * 1. 通信超时分级处理。
         * 注意：上电后如果一次都没收到过帧（例如树莓派还在开机），不做超时判断，
         *       否则下位机会在 1 秒后误判为「失联」而锁死。
         *       此时电机本来就是断电极、速度 0，不存在危险。
         */
        if (s_ever_rx && !s_estop_latch) {
            uint32_t age = s_tick_ms - s_last_rx_ms;
            if (age > COMM_LOST_MS) {
                fault_set(FAULT_COMM_TIMEOUT);
                emergency_stop();
            } else if (age > COMM_TIMEOUT_MS) {
                soft_stop();
            }
        }

        /* 2. 母线限流（电池持续 10A，阈值 8A） */
        if (estimate_bus_current_a() > BUS_CURRENT_LIMIT_A) {
            fault_set(FAULT_OVERCUR);
            s_target[MOTOR_LEFT]  = (int16_t)(s_target[MOTOR_LEFT] / 2);
            s_target[MOTOR_RIGHT] = (int16_t)(s_target[MOTOR_RIGHT] / 2);
        }

        /* 3. 堵转检测：占空比不低但编码器不动 */
        {
            bool stuck = false;
            for (uint8_t i = 0; i < MOTOR_NUM; i++) {
                int16_t d = s_cmd_duty[i];
                if ((d > STALL_DUTY_PCT * 10 || d < -STALL_DUTY_PCT * 10) &&
                    Encoder_GetDelta((motor_id_t)i) == 0) {
                    stuck = true;
                }
            }
            if (stuck) {
                s_stall_ms++;
                if (s_stall_ms > STALL_TIME_MS) {
                    fault_set(FAULT_STALL);
                    emergency_stop();
                }
            } else {
                s_stall_ms = 0;
            }
        }
    }
}

/** 速度环：200Hz */
static void Task_Motor(void *arg)
{
    (void)arg;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (s_state == ST_FAULT) {
            continue;
        }

        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            int16_t tgt = s_target[i];                       /* mm/s */
            int16_t fb  = Encoder_GetSpeedMMps((motor_id_t)i);
            float   err = (float)(tgt - fb);
            float   duty;

            /* 增量式 PI */
            static float integ[MOTOR_NUM];
            integ[i] += PID_KI * err;
            if (integ[i] > 400.0f)  { integ[i] = 400.0f; }
            if (integ[i] < -400.0f) { integ[i] = -400.0f; }
            duty = PID_KP * err + integ[i];

            if (duty > 1000.0f)  { duty = 1000.0f; }
            if (duty < -1000.0f) { duty = -1000.0f; }

            s_cmd_duty[i] = (int16_t)duty;
        }

        if (s_state == ST_RUN) {
            Motor_SetDuty(MOTOR_LEFT,  s_cmd_duty[MOTOR_LEFT]);
            Motor_SetDuty(MOTOR_RIGHT, s_cmd_duty[MOTOR_RIGHT]);
        } else if (s_state == ST_IDLE) {
            Motor_CoastAll();
        }
    }
}

/** 编码器采样：100Hz */
static void Task_Encoder(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        Encoder_Update(10);
    }
}

/** 通信：事件驱动 + 20Hz 上报 */
static void Task_Comm(void *arg)
{
    proto_frame_t f;
    (void)arg;

    for (;;) {
        if (xQueueReceive(s_q_rx, &f, pdMS_TO_TICKS(50)) == pdTRUE) {
            switch (f.cmd) {
            case CMD_SET_SPEED:
                if (f.len >= 4) {
                    s_target[MOTOR_LEFT]  = proto_get_i16(&f.data[0]);
                    s_target[MOTOR_RIGHT] = proto_get_i16(&f.data[2]);
                    if (s_state == ST_IDLE) {
                        Motor_PowerOn();        /* 先合闸，再使能 */
                        Motor_SetEnable(true);
                        s_state = ST_RUN;
                    }
                }
                break;

            case CMD_STOP:
                soft_stop();
                s_state = ST_IDLE;
                break;

            case CMD_ESTOP:
                s_estop_latch = true;
                fault_set(FAULT_ESTOP);
                emergency_stop();
                break;

            case CMD_CLEAR_ESTOP:
                /* 两个轮子都基本停住才允许复位（断电后靠惯性还会滑行几秒） */
                if (abs_i16(Encoder_GetSpeedMMps(MOTOR_LEFT)) < 50 &&
                    abs_i16(Encoder_GetSpeedMMps(MOTOR_RIGHT)) < 50) {
                    s_estop_latch = false;
                    s_fault = FAULT_NONE;
                    s_state = ST_IDLE;
                    s_last_rx_ms = s_tick_ms;   /* 避免清除后立刻又超时 */
                }
                break;

            case CMD_RESET_ODO:
                Encoder_ResetOdo();
                break;

            case CMD_HEARTBEAT:
                break;

            case CMD_QUERY:
                send_status();
                break;

            default:
                send_ack(f.cmd, 0x01);          /* 未知命令 */
                break;
            }
        }

        /* 20Hz 周期上报 */
        if ((s_tick_ms - s_last_report_ms) >= (1000u / STATUS_REPORT_HZ)) {
            s_last_report_ms = s_tick_ms;
            send_status();
        }
    }
}

/** 维护任务：喂看门狗 */
static void Task_Monitor(void *arg)
{
    (void)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* ---------------- 初始化 ---------------- */

void App_Init(void)
{
    proto_parser_init(&s_parser);

    s_last_rx_ms    = 0;
    s_ever_rx       = false;
    s_last_report_ms = 0;
    s_state         = ST_IDLE;
    s_fault         = FAULT_NONE;
    s_estop_latch   = false;

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        s_target[i]   = 0;
        s_cmd_duty[i] = 0;
    }

    Motor_Init();
    Encoder_Init();

    s_q_rx = xQueueCreate(8, sizeof(proto_frame_t));
    configASSERT(s_q_rx != NULL);

    (void)xTaskCreate(Task_Safety,  "safety",  STK_SAFETY,  NULL, PRIO_SAFETY,  &s_h_safety);
    (void)xTaskCreate(Task_Motor,   "motor",   STK_MOTOR,   NULL, PRIO_MOTOR,   &s_h_motor);
    (void)xTaskCreate(Task_Encoder, "encoder", STK_ENCODER, NULL, PRIO_ENCODER, NULL);
    (void)xTaskCreate(Task_Comm,    "comm",    STK_COMM,    NULL, PRIO_COMM,    NULL);
    (void)xTaskCreate(Task_Monitor, "monitor", STK_MONITOR, NULL, PRIO_MONITOR, NULL);
}
