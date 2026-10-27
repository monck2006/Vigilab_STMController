"""
encoder_calib.py - 里程标定工具

作用：用"实际走了多远"和"板子报了多少里程"对照，估算每轮编码器计数的修正值。

为什么要标定：里程精度由 编码器线数 × 减速比 × 轮径 三者共同决定，
      任何一项有偏差，车跑远了都会累积误差；C3 的建图和定位全靠它。

用法：
  python encoder_calib.py -p COM3 --distance 2000

  1) 把车放在地面上，轮胎划个记号，量出起点；
  2) 脚本先发"里程清零"；
  3) 用遥控或手推让车沿直线走一段（越远越准，建议 2 米以上）；
  4) 量出实际直线距离，输入脚本；
      5) 脚本给出每轮计数的初步修正参考值。

依赖：pip install pyserial
"""
import argparse
import re
import sys
import time
from pathlib import Path

import proto

try:
    import serial
except ImportError:
    serial = None

CONFIG_PATH = Path(__file__).resolve().parents[1] / "C1_Keil" / "Core" / "Inc" / "config.h"


def read_firmware_config():
    """读取固件当前轮径和每轮计数，避免工具里再维护一份旧参数。"""
    source = CONFIG_PATH.read_text(encoding="utf-8")
    wheel = re.search(r"^\s*#define\s+WHEEL_DIAMETER_MM\s+([0-9.]+)f?", source, re.M)
    counts = re.search(r"^\s*#define\s+ENCODER_COUNTS_PER_WHEEL_REV\s+(\d+)u?", source, re.M)
    if not wheel or not counts:
        raise RuntimeError(f"无法从 {CONFIG_PATH} 读取轮径或每轮编码器计数")
    return float(wheel.group(1)), int(counts.group(1))


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
        if d_real <= 0:
            print("实际距离必须大于 0 mm。")
            sys.exit(1)

        if dl == 0 or dr == 0 or dl * dr < 0:
            print("左右里程为 0 或方向相反：先检查编码器接线、方向和车轮是否转动。")
            sys.exit(1)

        avg = (abs(dl) + abs(dr)) / 2.0

        k = d_real / avg
        print(f"\n修正系数 k = {k:.4f}")
        if 0.98 <= k <= 1.02:
            print("偏差在 2% 以内，参数基本正确，可以不用改。")
        else:
            wheel_dia_mm, counts_per_rev = read_firmware_config()
            corrected_counts = counts_per_rev / k
            print(f"固件当前轮径：{wheel_dia_mm:.1f} mm")
            print(f"固件当前每轮计数：{counts_per_rev} counts/rev")
            print(f"建议实测并更新 ENCODER_COUNTS_PER_WHEEL_REV，初步参考值约 {corrected_counts:.0f}")
            print("轮径应按实物测量，不要为了修正编码器比例而随意改轮径。")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
