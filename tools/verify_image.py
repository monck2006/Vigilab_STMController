"""校验固件镜像：Intel HEX 校验和 + 向量表 + 地址范围"""
import sys
from pathlib import Path

HEX = Path(r"C:\Users\34983\Desktop\大创\C1_运动控制\C1_GCC\build\C1.hex")


def parse_hex(path):
    data = {}
    upper = 0
    bad = 0
    lines = 0
    for raw in path.read_text().splitlines():
        raw = raw.strip()
        if not raw.startswith(":"):
            continue
        lines += 1
        b = bytes.fromhex(raw[1:])
        if (sum(b) & 0xFF) != 0:
            bad += 1
        n, addr, typ = b[0], (b[1] << 8) | b[2], b[3]
        payload = b[4:4 + n]
        if typ == 0x00:
            for i, v in enumerate(payload):
                data[upper + addr + i] = v
        elif typ == 0x04:
            upper = ((payload[0] << 8) | payload[1]) << 16
    return data, lines, bad


def main():
    if not HEX.exists():
        print("找不到 C1.hex：", HEX)
        return 1
    data, lines, bad = parse_hex(HEX)
    lo, hi = min(data), max(data)

    print(f"HEX 记录数     : {lines}")
    print(f"校验和错误     : {bad}  {'[OK]' if bad == 0 else '[FAIL]'}")
    print(f"地址范围       : 0x{lo:08X} ~ 0x{hi:08X}  ({hi - lo + 1} 字节)")
    print(f"落在 Flash 区  : {'[OK]' if 0x08000000 <= lo and hi < 0x08100000 else '[FAIL]'}")

    def rd(a):
        return int.from_bytes(bytes(data.get(a + i, 0) for i in range(4)), "little")

    sp = rd(0x08000000)
    rst = rd(0x08000004)
    nmih = rd(0x08000008)
    hardf = rd(0x0800000C)

    print()
    print("--- 向量表（前 4 项）---")
    print(f"初始 SP        : 0x{sp:08X}  {'[OK] 指向 SRAM' if 0x20000000 <= sp <= 0x20020000 else '[FAIL]'}")
    for name, v in (("Reset_Handler", rst), ("NMI_Handler", nmih), ("HardFault", hardf)):
        ok = (v & 1) == 1 and 0x08000000 <= (v & ~1) < 0x08100000
        print(f"{name:15s}: 0x{v:08X}  {'[OK] Thumb 入口' if ok else '[FAIL]'}")

    uniq = len(set((rst & ~1, nmih & ~1, hardf & ~1)))
    print(f"入口互不相同   : {'[OK]' if uniq == 3 else '[注意] 有重复，检查启动文件'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
