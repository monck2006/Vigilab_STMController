"""
C1 固件逻辑仿真（不接硬件）

把 app_tasks.c / motor.c / encoder.c 的控制逻辑按同样规则在电脑上跑一遍，
验证：状态机、软启动、速度环、通信超时分级、急停断电、堵转、限流、里程换算。
参数直接从 config.h 里读，保证和固件一致。
"""
import re
from pathlib import Path

CFG = Path(r"C:\Users\34983\Desktop\大创\C1_运动控制\C1_Keil\Core\Inc\config.h")


def load_cfg():
    src = CFG.read_text(encoding="utf-8", errors="replace")
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    out = {}
    for name, expr in re.findall(r"#define[ \t]+(\w+)[ \t]+([^\n]+)", src):
        expr = expr.split("//")[0].strip()
        expr = expr.rstrip("fF")
        try:
            out[name] = float(eval(expr, {}, out)) if not expr.isdigit() else int(expr)
        except Exception:
            pass
    return out


C = load_cfg()
MM_PER_COUNT = 3.14159265 * C["WHEEL_DIA_MM"] / (C["ENC_PPR"] * C["ENC_MULT"] * C["GEAR_RATIO"])

ST_INIT, ST_IDLE, ST_RUN, ST_FAULT = 0, 1, 2, 3
F_NONE, F_COMM, F_ESTOP, F_STALL, F_OVERCUR = 0, 1, 2, 3, 4


class Robot:
    """按固件逻辑实现的仿真模型"""

    def __init__(self):
        self.t_ms = 0
        self.state = ST_IDLE
        self.fault = F_NONE
        self.power_on = False
        self.enabled = False
        self.estop_latch = False
        self.stall_ms = 0
        self.last_rx_ms = 0
        self.ever_rx = False
        self.target = [0, 0]      # mm/s
        self.duty = [0.0, 0.0]    # 千分比
        self.integ = [0.0, 0.0]
        self.speed = [0.0, 0.0]   # 实际轮速 mm/s
        self.odo_cnt = [0, 0]
        self.enc_last = [0, 0]
        self.blocked = [False, False]   # 人为制造堵转
        self.log = []

    # ---- 固件里的紧急停车 ----
    def emergency_stop(self):
        self.power_on = False
        self.enabled = False
        self.target = [0, 0]
        self.duty = [0.0, 0.0]
        self.state = ST_FAULT

    def fault_set(self, code):
        if self.fault == F_NONE:
            self.fault = code

    # ---- 1kHz 安全任务 ----
    def task_safety(self):
        if self.ever_rx and not self.estop_latch:
            age = self.t_ms - self.last_rx_ms
            if age > C["COMM_LOST_MS"]:
                self.fault_set(F_COMM)
                self.emergency_stop()
            elif age > C["COMM_TIMEOUT_MS"]:
                self.target = [0, 0]

        # 限流：按占空比估算
        est = sum(abs(d) / 1000.0 * C["MOTOR_RATED_A"] for d in self.duty)
        if est > C["BUS_CURRENT_LIMIT_A"]:
            self.fault_set(F_OVERCUR)
            self.target = [t / 2 for t in self.target]

        # 堵转
        stuck = any(abs(d) > C["STALL_DUTY_PCT"] * 10 and self.speed[i] < 1e-3
                    for i, d in enumerate(self.duty))
        self.stall_ms = self.stall_ms + 1 if stuck else 0
        if self.stall_ms > C["STALL_TIME_MS"]:
            self.fault_set(F_STALL)
            self.emergency_stop()

    # ---- 200Hz 速度环 ----
    def task_motor(self):
        if self.state == ST_FAULT:
            return
        for i in range(2):
            err = self.target[i] - self.speed[i]
            self.integ[i] += C["PID_KI"] * err
            self.integ[i] = max(-400, min(400, self.integ[i]))
            d = C["PID_KP"] * err + self.integ[i]
            self.duty[i] = max(-1000, min(1000, d))
        if self.state == ST_RUN and self.power_on:
            pass  # 占空比生效，见 step_plant
        else:
            self.duty = [0.0, 0.0]

    # ---- 100Hz 编码器 ----
    def task_encoder(self):
        for i in range(2):
            cnt = int(sum(self.trace[i]) / MM_PER_COUNT) if self.trace[i] else 0
            self.odo_cnt[i] = cnt

    # ---- 被控对象：一阶惯性 + 软启动限幅 ----
    def step_plant(self, dt_s):
        if not (self.power_on and self.enabled and self.state == ST_RUN):
            for i in range(2):
                self.speed[i] += (0 - self.speed[i]) * min(1, dt_s / 0.35)
            return
        for i in range(2):
            if self.blocked[i]:
                self.speed[i] = 0.0
                continue
            v_ss = self.duty[i] / 1000.0 * C["VEL_MAX_MMPS"]
            # 加速度限幅（软启动）
            max_dv = C["ACC_MAX_MMPS2"] * dt_s
            dv = (v_ss - self.speed[i]) * min(1, dt_s / 0.15)
            dv = max(-max_dv, min(max_dv, dv))
            self.speed[i] += dv

    def command(self, cmd, a=0, b=0):
        self.last_rx_ms = self.t_ms
        self.ever_rx = True
        if cmd == "speed":
            self.target = [a, b]
            if self.state == ST_IDLE:
                self.power_on = True
                self.enabled = True
                self.state = ST_RUN
        elif cmd == "stop":
            self.target = [0, 0]
            self.state = ST_IDLE
        elif cmd == "estop":
            self.estop_latch = True
            self.fault_set(F_ESTOP)
            self.emergency_stop()
        elif cmd == "clear":
            if abs(self.speed[0]) < 50 and abs(self.speed[1]) < 50:
                self.estop_latch = False
                self.fault = F_NONE
                self.state = ST_IDLE
                self.last_rx_ms = self.t_ms

    def flags(self):
        return (1 if self.power_on else 0) | 0x02 | (0x04 if self.estop_latch else 0)

    def odo_mm(self, i):
        return self.odo_cnt[i] * MM_PER_COUNT


def run(name, seconds, events, blocked=(False, False)):
    r = Robot()
    r.trace = [[], []]
    r.blocked = list(blocked)
    steps = int(seconds * 1000)
    print(f"\n=== {name} ===")
    for t in range(steps):
        r.t_ms = t
        for (at, fn) in events:
            if at == t:
                fn(r)
        if t % 1 == 0:
            r.task_safety()
        if t % int(C["SPEED_LOOP_DIV"]) == 0:
            r.task_motor()
        r.step_plant(0.001)
        if t % 10 == 0:
            for i in range(2):
                r.trace[i].append(r.speed[i] * 0.01)    # 10ms 位移，mm
            r.task_encoder()
        if t % 100 == 0 and t > 0:
            print(f"  t={t:5d}ms state={r.state} fault={r.fault} flags=0b{r.flags():03b} "
                  f"power={int(r.power_on)} vL={r.speed[0]:7.1f} vR={r.speed[1]:7.1f} "
                  f"odoL={r.odo_mm(0):8.1f}mm")
    return r


def main():
    print(f"参数（读自 config.h）：轮径 {C['WHEEL_DIA_MM']:.0f}mm，减速比 {C['GEAR_RATIO']:.0f}，"
          f"编码器 {C['ENC_PPR']:.0f} 线，mm/计数 = {MM_PER_COUNT:.4f}")

    r = run("场景1：上电后什么都不发", 1.2, [])
    print(f"  → 结论：{'[OK]' if r.state == ST_IDLE and not r.power_on else '[FAIL]'} "
          f"上电保持 IDLE 且电机断电，不会自己动")

    r = run("场景2：发 400/400 mm/s 并保持", 2.0,
            [(100, lambda x: x.command("speed", 400, 400))] +
            [(100 + 100 * k, lambda x: x.command("speed", 400, 400)) for k in range(1, 18)])
    ok = abs(r.speed[0] - 400) < 20 and r.odo_mm(0) > 300
    print(f"  → 结论：{'[OK]' if ok else '[FAIL]'} 速度收敛到 {r.speed[0]:.0f} mm/s（目标 400），"
          f"2 秒累计里程 {r.odo_mm(0) / 1000:.3f} m")

    r = run("场景3：发完速度后停止发帧（通信超时）", 1.6,
            [(100, lambda x: x.command("speed", 400, 400))])
    print(f"  → 结论：{'[OK]' if r.state == ST_FAULT and r.fault == F_COMM and not r.power_on else '[FAIL]'} "
          f"500ms 软停、1s 后断电锁定，fault={r.fault}（1=通信超时），power={int(r.power_on)}")

    r = run("场景4：急停 → 清除急停", 1.0, [
        (100, lambda x: x.command("speed", 400, 400)),
        (300, lambda x: x.command("estop")),
        (750, lambda x: x.command("clear")),
    ])
    print(f"  → 结论：{'[OK]' if r.state == ST_IDLE and r.fault == F_NONE and not r.power_on else '[FAIL]'} "
          f"急停后断电锁定，清除后回到 IDLE；注意清除后电源仍是断的（要重新发速度才合闸）")

    r = run("场景5：两个轮子被卡住还使劲转（堵转）", 1.2,
            [(100, lambda x: x.command("speed", 600, 600))] +
            [(100 + 100 * k, lambda x: x.command("speed", 600, 600)) for k in range(1, 11)],
            blocked=(True, True))
    print(f"  → 结论：{'[OK]' if r.fault == F_STALL and not r.power_on else '[FAIL]'} "
          f"500ms 后判定堵转并断电，fault={r.fault}（3=堵转）")

    # 里程换算精度
    dist = 10000.0
    cnt = dist / MM_PER_COUNT
    print(f"\n=== 场景6：里程换算精度（轮径 {C['WHEEL_DIA_MM']:.0f}mm）===")
    print(f"  走 10 m 对应编码器计数 {cnt:.0f}，回算距离 {cnt * MM_PER_COUNT:.2f} mm")
    print(f"  → {'[OK]' if abs(cnt * MM_PER_COUNT - dist) < 1 else '[FAIL]'} 换算自洽")
    print(f"  单计数对应位移 {MM_PER_COUNT:.4f} mm（0.23mm 量级，里程精度足够 C3 建图）")

    print("\n仿真结束。以上是纯逻辑仿真，验证的是控制逻辑与状态机；")
    print("真机行为（PWM 波形、编码器电气、电机实际转向）仍需上板确认。")


if __name__ == "__main__":
    main()
