"""
proto.py - C1 上下位机协议（Python 参考实现）

与 firmware/Core/Src/protocol.c 完全一致：
  帧：AA 55 | LEN | SEQ | CMD | DATA | CRC16(小端)
  LEN = SEQ + CMD + DATA 的字节数
  CRC16-Modbus：多项式 0xA001，初值 0xFFFF，覆盖 LEN..DATA
"""

SOF = bytes([0xAA, 0x55])
MAX_DATA = 32
MAX_LEN = MAX_DATA + 2

# 命令码
CMD_SET_SPEED = 0x01
CMD_STOP = 0x02
CMD_ESTOP = 0x03
CMD_CLEAR_ESTOP = 0x04
CMD_RESET_ODO = 0x05
CMD_SET_PID = 0x06
CMD_HEARTBEAT = 0x07
CMD_STATUS = 0x10
CMD_QUERY = 0x11
CMD_ACK = 0x7F

CMD_NAME = {
    CMD_SET_SPEED: "设置轮速", CMD_STOP: "软停", CMD_ESTOP: "急停",
    CMD_CLEAR_ESTOP: "清除急停", CMD_RESET_ODO: "里程清零", CMD_SET_PID: "设置PID",
    CMD_HEARTBEAT: "心跳", CMD_STATUS: "状态上报", CMD_QUERY: "查询", CMD_ACK: "应答",
}

STATE_NAME = {0x00: "INIT", 0x01: "IDLE", 0x02: "RUN", 0x03: "FAULT"}
FAULT_NAME = {
    0x00: "正常", 0x01: "通信超时停车", 0x02: "急停锁定", 0x03: "堵转",
    0x04: "过流限幅", 0x05: "电池欠压", 0x06: "编码器异常",
}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def pack(seq: int, cmd: int, data: bytes = b"") -> bytes:
    if len(data) > MAX_DATA:
        raise ValueError("数据域超长")
    body = bytes([len(data) + 2, seq & 0xFF, cmd & 0xFF]) + bytes(data)
    c = crc16(body)
    return SOF + body + bytes([c & 0xFF, c >> 8])


def pack_speed(seq: int, v_left: int, v_right: int) -> bytes:
    import struct
    return pack(seq, CMD_SET_SPEED, struct.pack("<hh", int(v_left), int(v_right)))


class Parser:
    """与 C 版本逐字节行为一致的状态机解析器"""

    def __init__(self):
        self.state = 0
        self.buf = bytearray(MAX_LEN + 2)
        self.idx = 0
        self.length = 0
        self.err_crc = 0
        self.err_len = 0
        self.err_sync = 0
        self.ok = 0

    def feed(self, byte: int):
        """返回 (seq, cmd, data) 或 None"""
        if self.state == 0:
            if byte == 0xAA:
                self.state = 1
        elif self.state == 1:
            if byte == 0x55:
                self.state = 2
            elif byte != 0xAA:
                self.err_sync += 1
                self.state = 0
        elif self.state == 2:
            if byte < 2 or byte > MAX_LEN:
                self.err_len += 1
                self.state = 0
            else:
                self.length = byte
                self.buf[0] = byte
                self.idx = 1
                self.state = 3
        elif self.state == 3:
            self.buf[self.idx] = byte
            self.idx += 1
            if self.idx >= self.length + 1:
                self.state = 4
        elif self.state == 4:
            self.buf[self.idx] = byte
            self.state = 5
        elif self.state == 5:
            self.buf[self.idx + 1] = byte
            rx = self.buf[self.idx] | (byte << 8)
            self.state = 0
            if rx != crc16(bytes(self.buf[: self.length + 1])):
                self.err_crc += 1
                return None
            self.ok += 1
            return (self.buf[1], self.buf[2], bytes(self.buf[3 : 1 + self.length]))
        return None


def parse_all(data: bytes):
    """从一段字节流里解析出所有合法帧"""
    p = Parser()
    frames = []
    for b in data:
        r = p.feed(b)
        if r:
            frames.append(r)
    return frames


def decode_status(data: bytes):
    """把状态帧的数据域解成可读 dict"""
    import struct
    if len(data) < 17:
        return None
    state, fault, vbus = data[0], data[1], struct.unpack_from("<H", data, 2)[0]
    vl, vr = struct.unpack_from("<hh", data, 4)
    ol, orr = struct.unpack_from("<ii", data, 8)
    return {
        "state": STATE_NAME.get(state, f"0x{state:02X}"),
        "fault": FAULT_NAME.get(fault, f"0x{fault:02X}"),
        "vbus_mV": vbus,
        "v_left_mmps": vl,
        "v_right_mmps": vr,
        "odo_left_mm": ol,
        "odo_right_mm": orr,
        "cnt": data[16],
    }


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)
