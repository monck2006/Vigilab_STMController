"""
encoder_calib.py - 里程标定工具

作用：用"实际走了多远"和"板子报了多少里程"对照，反推真实的比例系数，
      顺便确认编码器到底是 11 线还是 13 线。

为什么要标定：里程精度由 编码器线数 × 减速比 × 轮径 三者共同决定，
      任何一项有偏差，车跑远了都会累积误差；C3 的建图和定位全靠它。

用法：
  python encoder_calib.py -p COM3 --distance 2000

  1) 把车放在地面上，轮胎划个记号，量出起点；
  2) 脚本先发"里程清零"；
  3) 用遥控或手推让车沿直线走一段（越远越准，建议 2 米以上）；
  4) 量出实际直线距离，输入脚本；
  5) 脚本给出修正后的参数。

依赖：pip install pyserial
"""
import argparse
import sys
import time

import proto

try:
    import serial
except ImportError:
    serial = None

# 当前固件里 config.h 的设定值
ENC_PPR = 11
ENC_MULT = 4
GEAR_RATIO = 56.0
WHEEL_DIA_MM = 100.0


def read_odo(ser, timeout=2.0):
    """发查询并返回 (左里程, 右里程) mm"""
    parser = proto.Parser()
    seq = int(time.time()) & 0xFF
    ser.write(proto.pack(seq, proto.CMD_QUERY))
    ser.flush()

    t0 = time.time()
    while time.time() - t0 < timeout:
        for b in ser.read(256):
            r = parser.feed(b)
            if r and r[1] == proto.CMD_STATUS:
                st = proto.decode_status(r[2])
                if st:
                    return st["odo_left_mm"], st["odo_right_mm"]
    return None


def main():
    ap = argparse.ArgumentParser(description="C1 里程标定")
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--distance", type=float,
                    help="实际直线距离 mm；不给则进入交互模式")
    a = ap.parse_args()

    if serial is None:
        print("缺少 pyserial，请先执行：  pip install pyserial")
        sys.exit(2)

    ser = serial.Serial(a.port, a.baud, timeout=0.1)
    try:
        print("发『里程清零』……")
        ser.write(proto.pack(1, proto.CMD_RESET_ODO))
        ser.flush()
        time.sleep(0.3)

        base = read_odo(ser)
        if base is None:
            print("读不到状态帧：检查接线（PB10/PB11）、波特率和供电。")
            sys.exit(1)
        print(f"清零后基准里程：左 {base[0]} mm，右 {base[1]} mm")

        print("\n现在让机器人沿直线走 2 米以上，走完按回车。")
        input()
        now = read_odo(ser)
        if now is None:
            print("读不到状态帧。")
            sys.exit(1)
        dl = now[0] - base[0]
        dr = now[1] - base[1]
        print(f"板子报告：左 {dl} mm，右 {dr} mm")

        d_real = a.distance
        if d_real is None:
            d_real = float(input("请输入实际直线距离 (mm)："))

        avg = (dl + dr) / 2.0
        if avg <= 0:
            print("报告里程为 0 或负值：检查编码器接线（A/B 是否接反）与轮子是否打滑。")
            sys.exit(1)

        k = d_real / avg
        print(f"\n修正系数 k = {k:.4f}")
        if 0.98 <= k <= 1.02:
            print("偏差在 2% 以内，参数基本正确，可以不用改。")
        else:
            cur_mm_per_cnt = 3.14159265 * WHEEL_DIA_MM / (ENC_PPR * ENC_MULT * GEAR_RATIO)
            print(f"当前 MM_PER_COUNT = {cur_mm_per_cnt:.5f} mm/计数")
            print(f"应改为          = {cur_mm_per_cnt * k:.5f} mm/计数")
            print("\n两种改法（改其一即可，改完重新编译）：")
            print(f"  ① 按轮径修正：WHEEL_DIA_MM  {WHEEL_DIA_MM:.1f} -> {WHEEL_DIA_MM * k:.1f}")
            print(f"  ② 按编码器线数修正：ENC_PPR {ENC_PPR} -> {ENC_PPR / k:.2f}")
            print("     如果算出来接近整数（比如 12.8 接近 13），说明编码器实际是那个线数。")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
