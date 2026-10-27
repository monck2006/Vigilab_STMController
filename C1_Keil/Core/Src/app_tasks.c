/**
 * app_tasks.c - 应用层任务。
 *
 * Control 任务统一处理命令、安全条件、编码器采样和速度 PI，避免多个任务
 * 同时修改目标速度或占空比。USART 中断只解析并把完整帧放入队列；Telemetry
 * 任务是唯一的串口发送者。
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

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim7;
extern IWDG_HandleTypeDef hiwdg;

#define RX_QUEUE_LENGTH     8u
#define TX_QUEUE_LENGTH     4u
#define TX_PACKET_CAPACITY  48u
#define CONTROL_STACK_WORDS 512u
#define TX_STACK_WORDS      256u
#define CONTROL_PRIORITY    5u
#define TX_PRIORITY         2u

typedef struct {
    uint16_t length;
    uint8_t bytes[TX_PACKET_CAPACITY];
} tx_packet_t;

static QueueHandle_t s_rx_queue;
static QueueHandle_t s_tx_queue;
static TaskHandle_t s_control_task;
static proto_parser_t s_parser;
static proto_frame_t s_isr_frame;

/* 只由 TIM7 中断递增；接收中断也读取它来标记最后一帧时间。 */
static volatile uint32_t s_tick_ms;
static volatile uint32_t s_last_rx_ms;
static volatile bool s_have_rx;

/* 以下状态只由 Control 任务修改。 */
static uint8_t s_state = ST_IDLE;
static uint8_t s_fault = FAULT_NONE;
static bool s_estop_latched;
static bool s_comm_timeout_active;
static int16_t s_target_mmps[MOTOR_NUM];
static float s_ramp_mmps[MOTOR_NUM];
static float s_integral[MOTOR_NUM];
static float s_kp = SPEED_PI_KP;
static float s_ki_per_second = SPEED_PI_KI_PER_SECOND;
#if STALL_DETECT_ENABLE
static uint32_t s_stall_ms;
#endif
static uint32_t s_last_control_ms;
static uint32_t s_last_status_ms;
static uint32_t s_last_watchdog_ms;
static uint8_t s_tx_seq;

static int32_t abs_i16(int16_t value)
{
    return (value < 0) ? -(int32_t)value : (int32_t)value;
}

static int16_t clamp_speed(int16_t value)
{
    if (value > VEL_MAX_MMPS) { return VEL_MAX_MMPS; }
    if (value < -VEL_MAX_MMPS) { return -VEL_MAX_MMPS; }
    return value;
}

static float move_toward(float current, float target, float max_change)
{
    if (current < target) {
        float next = current + max_change;
        return (next > target) ? target : next;
    }
    if (current > target) {
        float next = current - max_change;
        return (next < target) ? target : next;
    }
    return current;
}

static bool queue_tx(uint8_t command, const uint8_t *data, uint8_t data_len)
{
    tx_packet_t packet;

    packet.length = proto_pack(packet.bytes, sizeof(packet.bytes), s_tx_seq++,
                               command, data, data_len);
    if (packet.length == 0u) {
        return false;
    }
    return xQueueSend(s_tx_queue, &packet, 0) == pdTRUE;
}

static void queue_ack(uint8_t acknowledged_command, uint8_t result)
{
    uint8_t data[2] = { acknowledged_command, result };
    (void)queue_tx(CMD_ACK, data, sizeof(data));
}

static void queue_status(void)
{
    uint8_t data[17];
    uint16_t unknown_voltage = 0xFFFFu;

    data[0] = s_state;
    data[1] = s_fault;
    data[2] = (uint8_t)(unknown_voltage & 0xFFu);
    data[3] = (uint8_t)(unknown_voltage >> 8);
    proto_put_i16(&data[4], Encoder_GetSpeedMMps(MOTOR_LEFT));
    proto_put_i16(&data[6], Encoder_GetSpeedMMps(MOTOR_RIGHT));
    proto_put_i32(&data[8], Encoder_GetOdoMM(MOTOR_LEFT));
    proto_put_i32(&data[12], Encoder_GetOdoMM(MOTOR_RIGHT));

    /* bit0=软件电源开关输出，bit1=驱动输出已允许，bit2=急停/锁定。 */
    data[16] = (uint8_t)((Motor_IsPowerOn() ? 0x01u : 0u) |
                         (Motor_IsEnabled() ? 0x02u : 0u) |
                         (s_estop_latched ? 0x04u : 0u));
    (void)queue_tx(CMD_STATUS, data, sizeof(data));
}

static void enter_fault(uint8_t fault_code)
{
    s_fault = fault_code;
    s_state = ST_FAULT;
    s_estop_latched = true;
    s_target_mmps[MOTOR_LEFT] = 0;
    s_target_mmps[MOTOR_RIGHT] = 0;
    s_ramp_mmps[MOTOR_LEFT] = 0.0f;
    s_ramp_mmps[MOTOR_RIGHT] = 0.0f;
    s_integral[MOTOR_LEFT] = 0.0f;
    s_integral[MOTOR_RIGHT] = 0.0f;

    /* 软件停机：PWM 归零并拉低 EN（若已配置）。这不是独立硬件急停。 */
    Motor_SetEnable(false);
    Motor_PowerCut();
}

static bool stopped_for_reset(void)
{
    return abs_i16(Encoder_GetSpeedMMps(MOTOR_LEFT)) <= STOP_SPEED_MMPS &&
           abs_i16(Encoder_GetSpeedMMps(MOTOR_RIGHT)) <= STOP_SPEED_MMPS;
}

static bool command_has_length(const proto_frame_t *frame, uint8_t expected)
{
    return frame->len == expected;
}

static void handle_command(const proto_frame_t *frame)
{
    int16_t left;
    int16_t right;

    switch (frame->cmd) {
    case CMD_SET_SPEED:
        if (!command_has_length(frame, 4u)) {
            queue_ack(frame->cmd, 2u);
            break;
        }
        if (s_estop_latched) {
            queue_ack(frame->cmd, 3u);
            break;
        }
        left = clamp_speed(proto_get_i16(&frame->data[0]));
        right = clamp_speed(proto_get_i16(&frame->data[2]));
        s_target_mmps[MOTOR_LEFT] = left;
        s_target_mmps[MOTOR_RIGHT] = right;
        s_fault = FAULT_NONE;
        s_comm_timeout_active = false;
        s_state = ST_RUN;
        Motor_PowerOn();
        Motor_SetEnable(true);
        queue_ack(frame->cmd, 0u);
        break;

    case CMD_STOP:
        if (!command_has_length(frame, 0u)) {
            queue_ack(frame->cmd, 2u);
            break;
        }
        if (!s_estop_latched && s_state != ST_IDLE) {
            s_target_mmps[MOTOR_LEFT] = 0;
            s_target_mmps[MOTOR_RIGHT] = 0;
            s_state = ST_STOPPING;
        }
        queue_ack(frame->cmd, 0u);
        break;

    case CMD_ESTOP:
        if (!command_has_length(frame, 0u)) {
            queue_ack(frame->cmd, 2u);
            break;
        }
        enter_fault(FAULT_ESTOP);
        queue_ack(frame->cmd, 0u);
        break;

    case CMD_CLEAR_ESTOP:
        if (!command_has_length(frame, 0u)) {
            queue_ack(frame->cmd, 2u);
            break;
        }
        if (!stopped_for_reset()) {
            queue_ack(frame->cmd, 4u);
            break;
        }
        s_estop_latched = false;
        s_comm_timeout_active = false;
        s_fault = FAULT_NONE;
        s_state = ST_IDLE;
        Motor_SetEnable(false);
        Motor_PowerCut();
        queue_ack(frame->cmd, 0u);
        break;

    case CMD_RESET_ODO:
        if (!command_has_length(frame, 0u)) {
            queue_ack(frame->cmd, 2u);
            break;
        }
        Encoder_ResetOdo();
        queue_ack(frame->cmd, 0u);
        break;

    case CMD_SET_PID:
        if (!command_has_length(frame, 4u)) {
            queue_ack(frame->cmd, 2u);
            break;
        }
        {
            int16_t kp_x1000 = proto_get_i16(&frame->data[0]);
            int16_t ki_x1000 = proto_get_i16(&frame->data[2]);
            if (kp_x1000 < 0 || kp_x1000 > 2000 ||
                ki_x1000 < 0 || ki_x1000 > 20000) {
                queue_ack(frame->cmd, 2u);
                break;
            }
            s_kp = (float)kp_x1000 / 1000.0f;
            s_ki_per_second = (float)ki_x1000 / 1000.0f;
            s_integral[MOTOR_LEFT] = 0.0f;
            s_integral[MOTOR_RIGHT] = 0.0f;
            queue_ack(frame->cmd, 0u);
        }
        break;

    case CMD_HEARTBEAT:
        if (!command_has_length(frame, 0u)) {
            queue_ack(frame->cmd, 2u);
        }
        break;

    case CMD_QUERY:
        if (!command_has_length(frame, 0u)) {
            queue_ack(frame->cmd, 2u);
        } else {
            queue_status();
        }
        break;

    default:
        queue_ack(frame->cmd, 1u);
        break;
    }
}

static void drain_rx_queue(void)
{
    proto_frame_t frame;
    while (xQueueReceive(s_rx_queue, &frame, 0) == pdTRUE) {
        handle_command(&frame);
    }
}

static void update_communication_safety(void)
{
    uint32_t now_ms;
    uint32_t last_rx_ms;
    uint32_t age_ms;
    bool have_rx;

    /* TIM7/USART3 都能更新这些值；取一致快照，避免毫秒计数跨界读出假超时。 */
    taskENTER_CRITICAL();
    now_ms = s_tick_ms;
    last_rx_ms = s_last_rx_ms;
    have_rx = s_have_rx;
    taskEXIT_CRITICAL();

    if (!have_rx || s_estop_latched) {
        return;
    }
    age_ms = now_ms - last_rx_ms;

    if (age_ms > COMM_LOST_MS && s_comm_timeout_active) {
        enter_fault(FAULT_COMM_TIMEOUT);
        return;
    }

    if (age_ms > COMM_TIMEOUT_MS &&
        (s_state == ST_RUN || s_state == ST_STOPPING)) {
        s_target_mmps[MOTOR_LEFT] = 0;
        s_target_mmps[MOTOR_RIGHT] = 0;
        s_state = ST_STOPPING;
        s_fault = FAULT_COMM_TIMEOUT;
        s_comm_timeout_active = true;
    } else if (age_ms <= COMM_TIMEOUT_MS) {
        s_comm_timeout_active = false;
        if (s_fault == FAULT_COMM_TIMEOUT) {
            s_fault = FAULT_NONE;
        }
    }
}

static void update_stall_safety(uint32_t dt_ms)
{
#if STALL_DETECT_ENABLE
    bool stalled = false;

    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        int16_t duty = Motor_GetDuty((motor_id_t)i);
        bool commanded = s_target_mmps[i] != 0;
        bool high_duty = (duty >= STALL_DUTY_PERMILLE) ||
                         (duty <= -STALL_DUTY_PERMILLE);
        if (commanded && high_duty && Encoder_GetDelta((motor_id_t)i) == 0) {
            stalled = true;
        }
    }

    if (stalled) {
        s_stall_ms += dt_ms;
        if (s_stall_ms >= STALL_TIME_MS) {
            enter_fault(FAULT_STALL);
        }
    } else {
        s_stall_ms = 0u;
    }
#else
    (void)dt_ms;
#endif
}

static int16_t run_pi(uint8_t index, int16_t target_mmps,
                      int16_t measured_mmps, float dt_seconds)
{
    float error = (float)target_mmps - (float)measured_mmps;
    float proposed_integral = s_integral[index] +
                              s_ki_per_second * error * dt_seconds;
    float output;

    if (proposed_integral > SPEED_PI_INTEGRAL_LIMIT) {
        proposed_integral = SPEED_PI_INTEGRAL_LIMIT;
    } else if (proposed_integral < -SPEED_PI_INTEGRAL_LIMIT) {
        proposed_integral = -SPEED_PI_INTEGRAL_LIMIT;
    }

    output = s_kp * error + proposed_integral;
    /* 触及输出上限且误差仍把输出推向饱和时，不继续积分。 */
    if (!((output > PWM_MAX_PERMILLE && error > 0.0f) ||
          (output < -PWM_MAX_PERMILLE && error < 0.0f))) {
        s_integral[index] = proposed_integral;
    } else {
        output = s_kp * error + s_integral[index];
    }

    if (output > PWM_MAX_PERMILLE) { output = PWM_MAX_PERMILLE; }
    if (output < -PWM_MAX_PERMILLE) { output = -PWM_MAX_PERMILLE; }
    return (int16_t)output;
}

static void run_speed_control(uint32_t dt_ms)
{
    float dt_seconds = (float)dt_ms / 1000.0f;
    float max_ramp_change = ACC_MAX_MMPS2 * dt_seconds;

    if (s_state == ST_FAULT || s_state == ST_IDLE) {
        s_ramp_mmps[MOTOR_LEFT] = 0.0f;
        s_ramp_mmps[MOTOR_RIGHT] = 0.0f;
        s_integral[MOTOR_LEFT] = 0.0f;
        s_integral[MOTOR_RIGHT] = 0.0f;
        Motor_SetEnable(false);
        return;
    }

    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        float desired = (s_state == ST_RUN) ?
                        (float)s_target_mmps[i] : 0.0f;
        int16_t measured = Encoder_GetSpeedMMps((motor_id_t)i);
        int16_t ramp_target;
        int16_t duty;

        s_ramp_mmps[i] = move_toward(s_ramp_mmps[i], desired,
                                     max_ramp_change);
        ramp_target = (int16_t)s_ramp_mmps[i];
        if (ramp_target == 0 && abs_i16(measured) <= STOP_SPEED_MMPS) {
            s_integral[i] = 0.0f;
            duty = 0;
        } else {
            duty = run_pi(i, ramp_target, measured, dt_seconds);
        }
        Motor_SetDuty((motor_id_t)i, duty);
    }

    if (s_state == ST_STOPPING &&
        s_ramp_mmps[MOTOR_LEFT] == 0.0f &&
        s_ramp_mmps[MOTOR_RIGHT] == 0.0f && stopped_for_reset()) {
        Motor_SetEnable(false);
        s_integral[MOTOR_LEFT] = 0.0f;
        s_integral[MOTOR_RIGHT] = 0.0f;
        s_state = ST_IDLE;
    }
}

static void Task_Control(void *argument)
{
    (void)argument;
    s_last_control_ms = s_tick_ms;
    s_last_status_ms = s_tick_ms;
    s_last_watchdog_ms = s_tick_ms;

    /* 定时器在调度器启动后再开，避免调度器启动前调用 FreeRTOS ISR API。 */
    App_StartUartRx();
    HAL_TIM_Base_Start_IT(&htim7);

    for (;;) {
        uint32_t now_ms;
        uint32_t dt_ms;

        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        now_ms = s_tick_ms;

        drain_rx_queue();
        update_communication_safety();

        dt_ms = now_ms - s_last_control_ms;
        if (dt_ms >= CONTROL_PERIOD_MS) {
            s_last_control_ms = now_ms;
            Encoder_Update(dt_ms);
            update_stall_safety(dt_ms);
            run_speed_control(dt_ms);
        }

        if ((now_ms - s_last_status_ms) >= STATUS_REPORT_PERIOD_MS) {
            s_last_status_ms = now_ms;
            queue_status();
        }

        if ((now_ms - s_last_watchdog_ms) >= 500u) {
            s_last_watchdog_ms = now_ms;
            (void)HAL_IWDG_Refresh(&hiwdg);
        }
    }
}

static void Task_Telemetry(void *argument)
{
    tx_packet_t packet;
    (void)argument;

    for (;;) {
        if (xQueueReceive(s_tx_queue, &packet, portMAX_DELAY) == pdTRUE &&
            packet.length > 0u) {
            (void)HAL_UART_Transmit(&huart3, packet.bytes, packet.length,
                                    UART_TX_TIMEOUT_MS);
        }
    }
}

void App_OnRxByte(uint8_t byte)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (proto_parser_feed(&s_parser, byte, &s_isr_frame)) {
        s_last_rx_ms = s_tick_ms;
        s_have_rx = true;
        (void)xQueueSendFromISR(s_rx_queue, &s_isr_frame,
                                &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void App_Tick1kHz(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    ++s_tick_ms;
    if (s_control_task != NULL) {
        vTaskNotifyGiveFromISR(s_control_task, &higher_priority_task_woken);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void App_Init(void)
{
    BaseType_t control_created;
    BaseType_t telemetry_created;

    proto_parser_init(&s_parser);
    s_tick_ms = 0u;
    s_last_rx_ms = 0u;
    s_have_rx = false;
    s_state = ST_IDLE;
    s_fault = FAULT_NONE;
    s_estop_latched = false;
    s_comm_timeout_active = false;

    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        s_target_mmps[i] = 0;
        s_ramp_mmps[i] = 0.0f;
        s_integral[i] = 0.0f;
    }

    Motor_Init();
    Encoder_Init();

    s_rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(proto_frame_t));
    s_tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(tx_packet_t));
    if (s_rx_queue == NULL || s_tx_queue == NULL) {
        for (;;) { }
    }

    control_created = xTaskCreate(Task_Control, "control", CONTROL_STACK_WORDS,
                                  NULL, CONTROL_PRIORITY, &s_control_task);
    telemetry_created = xTaskCreate(Task_Telemetry, "telemetry", TX_STACK_WORDS,
                                    NULL, TX_PRIORITY, NULL);
    if (control_created != pdPASS || telemetry_created != pdPASS) {
        for (;;) { }
    }
}
