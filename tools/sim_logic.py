"""简化的轮速 PI 演示模型，不是固件测试或电机电气仿真。

用法：python tools/sim_logic.py

参数从 C1_Keil/Core/Inc/config.h 读取。模型只展示目标斜坡、PI、通信超时
和软停的大致关系；电机惯量用一阶近似，不能用它判定真机稳定性或安全性。
"""
import re
from pathlib import Path

CONFIG = Path(__file__).resolve().parents[1] / "C1_Keil" / "Core" / "Inc" / "config.h"


def load_config():
    source = CONFIG.read_text(encoding="utf-8")
    values = {}
    names = (
        "WHEEL_DIAMETER_MM", "ENCODER_COUNTS_PER_WHEEL_REV", "CONTROL_PERIOD_MS",
        "VEL_MAX_MMPS", "ACC_MAX_MMPS2", "PWM_MAX_PERMILLE",
        "SPEED_PI_KP", "SPEED_PI_KI_PER_SECOND", "SPEED_PI_INTEGRAL_LIMIT",
        "COMM_TIMEOUT_MS", "COMM_LOST_MS", "STOP_SPEED_MMPS",
    )
    for name in names:
        match = re.search(
            rf"^\s*#define\s+{name}\s+([+-]?[0-9]+(?:\.[0-9]+)?)f?u?\b",
            source, re.M,
        )
        if not match:
            raise RuntimeError(f"config.h 缺少简单数值参数：{name}")
        raw = match.group(1)
        values[name] = float(raw) if "." in raw else int(raw)
    return values


C = load_config()
COUNTS_PER_REV = C["ENCODER_COUNTS_PER_WHEEL_REV"]
MM_PER_COUNT = 3.14159265 * C["WHEEL_DIAMETER_MM"] / COUNTS_PER_REV
CONTROL_MS = int(C["CONTROL_PERIOD_MS"])
STATES = {1: "IDLE", 2: "RUN", 3: "FAULT", 4: "STOPPING"}


def move_toward(current, target, amount):
    if current < target:
        return min(current + amount, target)
    if current > target:
        return max(current - amount, target)
    return current


class RobotModel:
    def __init__(self):
        self.state = 1
        self.fault = 0
        self.latched = False
        self.comm_timeout_active = False
        self.have_rx = False
        self.last_rx_ms = 0
        self.target = [0, 0]
        self.ramp = [0.0, 0.0]
        self.integral = [0.0, 0.0]
        self.duty = [0.0, 0.0]
        self.speed = [0.0, 0.0]
        self.measured = [0, 0]
        self.encoder_fraction = [0.0, 0.0]
        self.encoder_count = [0, 0]
        self.previous_count = [0, 0]
        self.distance_mm = [0.0, 0.0]
        self.blocked = [False, False]
        self.now_ms = 0
        self.last_control_ms = 0

    def command(self, name, left=0, right=0):
        self.have_rx = True
        self.last_rx_ms = self.now_ms
        if name == "speed" and not self.latched:
            limit = C["VEL_MAX_MMPS"]
            self.target = [max(-limit, min(limit, left)),
                           max(-limit, min(limit, right))]
            self.state = 2
            self.fault = 0
            self.comm_timeout_active = False
        elif name == "stop" and not self.latched and self.state != 1:
            self.target = [0, 0]
            self.state = 4
        elif name == "estop":
            self._fault(2)
        elif name == "clear" and self.latched:
            if all(abs(v) <= C["STOP_SPEED_MMPS"] for v in self.measured):
                self.latched = False
                self.comm_timeout_active = False
                self.fault = 0
                self.state = 1

    def _fault(self, code):
        self.fault = code
        self.state = 3
        self.latched = True
        self.target = [0, 0]
        self.ramp = [0.0, 0.0]
        self.integral = [0.0, 0.0]
        self.duty = [0.0, 0.0]

    def _safety_step(self):
        if not self.have_rx or self.latched:
            return
        age = self.now_ms - self.last_rx_ms
        if age > C["COMM_LOST_MS"] and self.comm_timeout_active:
            self._fault(1)
        elif age > C["COMM_TIMEOUT_MS"] and self.state in (2, 4):
            self.target = [0, 0]
            self.state = 4
            self.fault = 1
            self.comm_timeout_active = True
        elif age <= C["COMM_TIMEOUT_MS"]:
            self.comm_timeout_active = False
            if self.fault == 1:
                self.fault = 0

    def _encoder_step(self, dt_ms):
        for i in range(2):
            self.encoder_fraction[i] += self.speed[i] * dt_ms / 1000.0 / MM_PER_COUNT
            whole = int(self.encoder_fraction[i])
            self.encoder_fraction[i] -= whole
            self.encoder_count[i] += whole
            delta = self.encoder_count[i] - self.previous_count[i]
            self.previous_count[i] = self.encoder_count[i]
            self.measured[i] = int(delta * MM_PER_COUNT * 1000.0 / dt_ms)
            self.distance_mm[i] = self.encoder_count[i] * MM_PER_COUNT

    def _pi_step(self, dt_ms):
        dt = dt_ms / 1000.0
        max_change = C["ACC_MAX_MMPS2"] * dt
        if self.state in (1, 3):
            self.ramp = [0.0, 0.0]
            self.integral = [0.0, 0.0]
            self.duty = [0.0, 0.0]
            return

        for i in range(2):
            goal = self.target[i] if self.state == 2 else 0
            self.ramp[i] = move_toward(self.ramp[i], goal, max_change)
            error = int(self.ramp[i]) - self.measured[i]
            proposed = self.integral[i] + C["SPEED_PI_KI_PER_SECOND"] * error * dt
            bound = C["SPEED_PI_INTEGRAL_LIMIT"]
            proposed = max(-bound, min(bound, proposed))
            output = C["SPEED_PI_KP"] * error + proposed
            pwm_limit = C["PWM_MAX_PERMILLE"]
            if not ((output > pwm_limit and error > 0) or
                    (output < -pwm_limit and error < 0)):
                self.integral[i] = proposed
            else:
                output = C["SPEED_PI_KP"] * error + self.integral[i]
            self.duty[i] = max(-pwm_limit, min(pwm_limit, output))

        if (self.state == 4 and all(value == 0 for value in self.ramp) and
                all(abs(v) <= C["STOP_SPEED_MMPS"] for v in self.measured)):
            self.state = 1
            self.duty = [0.0, 0.0]
            self.integral = [0.0, 0.0]

    def _plant_step(self):
        dt = 0.001
        for i in range(2):
            if self.blocked[i]:
                self.speed[i] = 0.0
            elif self.state in (2, 4):
                steady_speed = self.duty[i] / C["PWM_MAX_PERMILLE"] * C["VEL_MAX_MMPS"]
                self.speed[i] += (steady_speed - self.speed[i]) * min(1.0, dt / 0.15)
            else:
                self.speed[i] += (0.0 - self.speed[i]) * min(1.0, dt / 0.35)

    def step(self, duration_ms):
        for _ in range(duration_ms):
            self.now_ms += 1
            self._safety_step()
            dt = self.now_ms - self.last_control_ms
            if dt >= CONTROL_MS:
                self.last_control_ms = self.now_ms
                self._encoder_step(dt)
                self._pi_step(dt)
            self._plant_step()

    def show(self, label):
        print(f"{label}: {STATES[self.state]}, fault={self.fault}, "
              f"target={self.target}, measured={self.measured}, "
              f"PWM={[round(x) for x in self.duty]}, "
              f"odo_mm={[round(x) for x in self.distance_mm]}")


def main():
    print("简化教学模型；不是固件测试，也不能证明真机安全。")
    print(f"轮径 {C['WHEEL_DIAMETER_MM']} mm，暂定 {COUNTS_PER_REV} count/rev，"
          f"单计数约 {MM_PER_COUNT:.3f} mm，速度环 {1000 / CONTROL_MS:.0f} Hz")

    robot = RobotModel()
    robot.step(100)
    robot.show("上电待机")
    robot.command("speed", 200, 200)
    for _ in range(8):
        robot.step(100)
        robot.command("speed", 200, 200)  # 模拟上位机定期发速度/心跳
    robot.show("持续发送 200 mm/s")
    robot.command("stop")
    for _ in range(15):
        robot.step(100)
        robot.command("heartbeat")
    robot.show("软停完成后")

    lost = RobotModel()
    lost.step(100)
    lost.command("speed", 200, 200)
    lost.step(1200)
    lost.show("发送速度后串口失联")


if __name__ == "__main__":
    main()
