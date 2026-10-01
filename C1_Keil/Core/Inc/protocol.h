/**
 * protocol.h - 上下位机通信协议（详见 docs/02_上下位机通信协议.md）
 *
 * 帧格式：
 *   AA 55 | LEN | SEQ | CMD | DATA[LEN-2] | CRC16(小端)
 *   LEN = SEQ + CMD + DATA 的字节数
 *   CRC 覆盖 LEN..DATA，多项式 0xA001，初值 0xFFFF
 */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PROTO_SOF0          0xAAu
#define PROTO_SOF1          0x55u
#define PROTO_MAX_DATA      32u                 /* 单帧最大数据域 */
#define PROTO_MAX_LEN       (PROTO_MAX_DATA + 2u) /* +SEQ +CMD */

/* ---- 命令码 ---- */
enum {
    CMD_SET_SPEED   = 0x01, /* i16 v_left, i16 v_right  (mm/s) */
    CMD_STOP        = 0x02, /* 软停 */
    CMD_ESTOP       = 0x03, /* 急停并锁定 */
    CMD_CLEAR_ESTOP = 0x04, /* 清除急停 */
    CMD_RESET_ODO   = 0x05, /* 里程计清零 */
    CMD_SET_PID     = 0x06, /* i16 kp_x1000, i16 ki_x1000 */
    CMD_HEARTBEAT   = 0x07, /* 心跳 */

    CMD_STATUS      = 0x10, /* 下位机 → 上位机：状态上报 */
    CMD_QUERY       = 0x11, /* 上位机查询，下位机立即回 CMD_STATUS */
    CMD_ACK         = 0x7F  /* u8 ack_cmd, u8 code */
};

/* ---- 运行状态 ---- */
enum {
    ST_INIT  = 0x00,
    ST_IDLE  = 0x01,
    ST_RUN   = 0x02,
    ST_FAULT = 0x03
};

/* ---- 故障码 ---- */
enum {
    FAULT_NONE         = 0x00,
    FAULT_COMM_TIMEOUT = 0x01,
    FAULT_ESTOP        = 0x02,
    FAULT_STALL        = 0x03,
    FAULT_OVERCUR      = 0x04,
    FAULT_UVLO         = 0x05,
    FAULT_ENCODER      = 0x06
};

typedef struct {
    uint8_t seq;
    uint8_t cmd;
    uint8_t len;                    /* 数据域长度 */
    uint8_t data[PROTO_MAX_DATA];
} proto_frame_t;

typedef struct {
    uint8_t  state;                 /* 解析状态机 */
    uint8_t  idx;
    uint8_t  len;
    uint8_t  buf[PROTO_MAX_LEN];    /* LEN..DATA */
    uint32_t ok_cnt;
    uint32_t err_crc;
    uint32_t err_len;
    uint32_t err_sync;
} proto_parser_t;

/* ---- CRC 与字节序辅助 ---- */
uint16_t proto_crc16(const uint8_t *buf, uint16_t len);
int16_t  proto_get_i16(const uint8_t *p);   /* 小端读 */
int32_t  proto_get_i32(const uint8_t *p);
void     proto_put_i16(uint8_t *p, int16_t v);
void     proto_put_i32(uint8_t *p, int32_t v);

/* ---- 解析与打包 ---- */
void proto_parser_init(proto_parser_t *p);

/**
 * 逐字节投喂。收到一个完整且校验正确的帧时返回 true，并填充 out。
 * 该函数可在串口中断里调用（纯状态机，无阻塞、无动态内存）。
 */
bool proto_parser_feed(proto_parser_t *p, uint8_t byte, proto_frame_t *out);

/**
 * 打包一帧到 out，返回整帧长度（含帧头与 CRC）。
 * out_size 不足时返回 0。
 */
uint16_t proto_pack(uint8_t *out, uint16_t out_size,
                    uint8_t seq, uint8_t cmd,
                    const uint8_t *data, uint8_t len);

#endif /* __PROTOCOL_H */
