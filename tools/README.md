# C1 工具集

| 文件 | 用途 |
| --- | --- |
| `proto.py` | 协议的 Python 参考实现，和固件里的 C 版本行为一致 |
| `serial_tester.py` | 串口测试：查状态、发速度、软停、急停、里程清零；含无硬件自检 |
| `encoder_calib.py` | 里程标定：用实际距离反推参数，顺便确认编码器线数 |

## 准备

```bash
pip install pyserial
```

接线：USB 转 TTL 的 TX 接 PB11（STM32 的 RX），RX 接 PB10（STM32 的 TX），GND 共地。
注意模块要选 **3.3V 电平**的那种，不要用 5V 的；STM32 侧不要用 USB 供电时还接 12V 电机电源。

## 常用命令

```bash
python serial_tester.py selftest              # 不用接硬件，先确认工具本身正常
python serial_tester.py -p COM3 monitor       # 看板子每 50ms 上报的状态
python serial_tester.py -p COM3 speed 200 200 # 左右轮各 200 mm/s
python serial_tester.py -p COM3 stop
python serial_tester.py -p COM3 estop
python serial_tester.py -p COM3 clear
python encoder_calib.py -p COM3 --distance 2000
```

## 第一次联调的顺序（建议）

1. `selftest` → 确认电脑端工具正常；
2. 板子只接 USB 供电、不接电机，`monitor` → 应该每 50ms 看到一帧状态；
3. 手转右轮，看"左/右里程"是否跟着变、方向对不对；
4. 接上电机电源，`speed 100 100` → 两个轮子应缓慢同向转动（有软启动）；
5. `estop` → 立即刹停并锁定，`clear` 才能恢复；
6. 最后做 `encoder_calib` 标定，把修正值写回 `config.h` 重新编译。
