/**
 * protocol.c - 协议实现：CRC16、状态机解析、打包
 */
#include "protocol.h"

/* ---------------- CRC16-Modbus (poly 0xA001, init 0xFFFF) ---------------- */
uint16_t proto_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ---------------- 小端读写 ---------------- */
int16_t proto_get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int32_t proto_get_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

void proto_put_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

void proto_put_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* ---------------- 解析状态机 ---------------- */
enum {
    PS_SOF0 = 0,
    PS_SOF1,
    PS_LEN,
    PS_PAYLOAD,
    PS_CRC_LO,
    PS_CRC_HI
};

void proto_parser_init(proto_parser_t *p)
{
    p->state = PS_SOF0;
    p->idx = 0;
    p->len = 0;
    p->ok_cnt = 0;
    p->err_crc = 0;
    p->err_len = 0;
    p->err_sync = 0;
}

bool proto_parser_feed(proto_parser_t *p, uint8_t byte, proto_frame_t *out)
{
    switch (p->state) {
    case PS_SOF0:
        if (byte == PROTO_SOF0) {
            p->state = PS_SOF1;
        }
        break;

    case PS_SOF1:
        if (byte == PROTO_SOF1) {
            p->state = PS_LEN;
        } else if (byte == PROTO_SOF0) {
            /* 连续 0xAA，仍可能是一帧的开头 */
        } else {
            p->err_sync++;
            p->state = PS_SOF0;
        }
        break;

    case PS_LEN:
        /* LEN 至少包含 SEQ+CMD，且不超过缓冲上限 */
        if (byte < 2u || byte > PROTO_MAX_LEN) {
            p->err_len++;
            p->state = PS_SOF0;
        } else {
            p->len = byte;
            p->buf[0] = byte;
            p->idx = 1;
            p->state = PS_PAYLOAD;
        }
        break;

    case PS_PAYLOAD:
        p->buf[p->idx++] = byte;
        if (p->idx >= (uint8_t)(p->len + 1u)) {
            p->state = PS_CRC_LO;
        }
        break;

    case PS_CRC_LO:
        p->buf[p->idx] = byte;              /* 暂存低字节 */
        p->state = PS_CRC_HI;
        break;

    case PS_CRC_HI: {
        p->buf[p->idx + 1u] = byte;
        uint16_t rx_crc = (uint16_t)(p->buf[p->idx] | ((uint16_t)byte << 8));
        uint16_t calc   = proto_crc16(p->buf, (uint16_t)(p->len + 1u));
        p->state = PS_SOF0;
        if (rx_crc != calc) {
            p->err_crc++;
            break;
        }
        /* 解析成功 */
        out->seq = p->buf[1];
        out->cmd = p->buf[2];
        out->len = (uint8_t)(p->len - 2u);
        for (uint8_t i = 0; i < out->len && i < PROTO_MAX_DATA; i++) {
            out->data[i] = p->buf[3u + i];
        }
        p->ok_cnt++;
        return true;
    }

    default:
        p->state = PS_SOF0;
        break;
    }
    return false;
}

/* ---------------- 打包 ---------------- */
uint16_t proto_pack(uint8_t *out, uint16_t out_size,
                    uint8_t seq, uint8_t cmd,
                    const uint8_t *data, uint8_t len)
{
    uint16_t need;

    if (len > PROTO_MAX_DATA) {
        return 0;
    }
    need = (uint16_t)(2u + 1u + 1u + 1u + len + 2u); /* SOF+LEN+SEQ+CMD+DATA+CRC */
    if (out_size < need) {
        return 0;
    }

    out[0] = PROTO_SOF0;
    out[1] = PROTO_SOF1;
    out[2] = (uint8_t)(len + 2u);       /* LEN = SEQ + CMD + DATA */
    out[3] = seq;
    out[4] = cmd;
    for (uint8_t i = 0; i < len; i++) {
        out[5u + i] = data ? data[i] : 0u;
    }

    {
        uint16_t crc = proto_crc16(&out[2], (uint16_t)(len + 3u));
        out[5u + len]      = (uint8_t)(crc & 0xFF);
        out[5u + len + 1u] = (uint8_t)((crc >> 8) & 0xFF);
    }
    return need;
}
