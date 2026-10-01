"""
serial_tester.py - 电脑端串口测试工具（C1 协议）

用途：STM32 烧好程序后，用 USB 转 TTL 直接接 USART3（PB10/PB11）就能验证通信，
      不需要等树莓派就绪。

用法：
  python serial_tester.py selftest                 # 无硬件自检，验证协议实现
  python serial_tester.py -p COM3 monitor          # 被动监听状态上报（20Hz）
  python serial_tester.py -p COM3 status           # 查询一次状态
  python serial_tester.py -p COM3 speed 200 200    # 左右轮各 200 mm/s
  python serial_tester.py -p COM3 stop
  python serial_tester.py -p COM3 estop
  python serial_tester.py -p COM3 clear
  python serial_tester.py -p COM3 reset-odo

依赖：pip install pyserial（只有 selftest 模式不需要）
"""
import argparse
import sys
import time

import proto

try:
    import serial  # pyserial
except ImportError:
    serial = None


# 已用 C 实现与 Python 实现双向核对过的测试向量
VEC_SPEED = "AA 55 06 01 01 C8 00 C8 00 28 B1"
VEC_STATUS = ("AA 55 13 02 10 02 00 2E 30 C8 00 C8 00 00 68 0D 00 "
              "00 00 00 00 00 F3 EE")


def selftest() -> int:
    """不接硬件，验证协议实现与文档里的测试向量一致"""
    ok = True

    f = proto.pack_speed(1, 200, 200)
    got = proto.hexs(f)
    print(f"打包 设置速度 200/200 : {got}")
    print(f"文档测试向量            : {VEC_SPEED}")
    if got != VEC_SPEED:
        print("  [FAIL] 与测试向量不一致")
        ok = False
    else:
        print("  [OK]")

    vec_status = bytes.fromhex(VEC_STATUS.replace(" ", ""))
    frames = proto.parse_all(vec_status)
    if len(frames) != 1:
        print("  [FAIL] 状态向量解析失败")
        ok = False
    else:
        seq, cmd, data = frames[0]
        st = proto.decode_status(data)
        print(f"解析 状态帧            : seq={seq} cmd=0x{cmd:02X} {st}")
        if st["vbus_mV"] != 12334 or st["odo_left_mm"] != 878592:
            print("  [FAIL] 字段解码不正确")
            ok = False
        else:
            print("  [OK]")

    bad = bytearray(vec_status)
    bad[7] ^= 0x01
    if proto.parse_all(bytes(bad)):
        print("  [FAIL] 翻转一个字节后仍被接受（CRC 无效）")
        ok = False
    else:
        print("校验 单字节翻转         : [OK] 已检出 CRC 错误")

    p = proto.Parser()
    got2 = None
    for b in b"\x11\x22\x33" + vec_status:
        r = p.feed(b)
        if r:
            got2 = r
    if got2 is None:
        print("  [FAIL] 垃圾字节后未能重新同步")
        ok = False
    else:
        print("校验 帧前混入垃圾字节   : [OK] 已自动重同步")

    print("\n自检结果：" + ("全部通过，协议实现与固件一致" if ok else "存在失败项"))
    return 0 if ok else 1


def open_port(port: str, baud: int):
    if serial is None:
        print("缺少 pyserial，请先执行：  pip install pyserial")
        sys.exit(2)
    return serial.Serial(port, baud, timeout=0.1)


def send(ser, frame: bytes):
    ser.write(frame)
    ser.flush()
    print("已发送: " + proto.hexs(frame))


def read_frames(ser, seconds: float):
    """读取一段时间内的所有合法帧"""
    parser = proto.Parser()
    frames = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = ser.read(256)
        for b in chunk:
            r = parser.feed(b)
            if r:
                frames.append(r)
    return frames


def print_frames(frames):
    for seq, cmd, data in frames:
        name = proto.CMD_NAME.get(cmd, f"0x{cmd:02X}")
        if cmd == proto.CMD_STATUS:
            st = proto.decode_status(data)
            if st:
                print(f"[{name}] 状态={st['state']:<5} 故障={st['fault']:<8} "
                      f"电压={st['vbus_mV']/1000:.2f}V "
                      f"左轮={st['v_left_mmps']:>5}mm/s 右轮={st['v_right_mmps']:>5}mm/s "
                      f"左里程={st['odo_left_mm']:>8}mm 右里程={st['odo_right_mm']:>8}mm")
        else:
            print(f"[{name}] seq={seq} data={proto.hexs(data)}")


def main():
    ap = argparse.ArgumentParser(description="C1 运动控制串口测试工具")
    ap.add_argument("-p", "--port", help="串口号，例如 COM3")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("action", help="selftest / monitor / status / speed / stop / estop / clear / reset-odo")
    ap.add_argument("args", nargs="*", help="speed 命令需要两个数：左轮 右轮 (mm/s)")
    a = ap.parse_args()

    if a.action == "selftest":
        sys.exit(selftest())

    if a.action == "monitor":
        ser = open_port(a.port, a.baud)
        print("监听中，Ctrl+C 退出……")
        try:
            while True:
                print_frames(read_frames(ser, 0.5))
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
        return

    seq = int(time.time()) & 0xFF
    if a.action == "status":
        frame = proto.pack(seq, proto.CMD_QUERY)
    elif a.action == "speed":
        if len(a.args) < 2:
            print("用法： speed <左轮 mm/s> <右轮 mm/s>")
            sys.exit(1)
        frame = proto.pack_speed(seq, int(a.args[0]), int(a.args[1]))
    elif a.action == "stop":
        frame = proto.pack(seq, proto.CMD_STOP)
    elif a.action == "estop":
        frame = proto.pack(seq, proto.CMD_ESTOP)
    elif a.action == "clear":
        frame = proto.pack(seq, proto.CMD_CLEAR_ESTOP)
    elif a.action == "reset-odo":
        frame = proto.pack(seq, proto.CMD_RESET_ODO)
    else:
        print("未知动作：" + a.action)
        sys.exit(1)

    ser = open_port(a.port, a.baud)
    try:
        send(ser, frame)
        print_frames(read_frames(ser, 1.0))
    finally:
        ser.close()


if __name__ == "__main__":
    main()
