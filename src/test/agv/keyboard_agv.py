"""
keyboard_agv.py — 用键盘 API 驱动 Cosys-AirSim AGV (Husky / SkidVehicle)

差速车 (skid-steer) 的控制比轿车简单:
    throttle 直接控速, 给负即后退, 不需要切倒挡.
    steering 控制左右轮速差.

按键:
    W / S       前进 / 后退
    A / D       左转 / 右转
    Q / E       原地左旋 / 原地右旋
    Space       手刹
    R           reset
    Esc         退出

依赖: pynput (apt: python3-pynput)

实现说明:
    msgpack-rpc-python 在 Python 3.12 + 子线程里会因为缺少 asyncio
    event loop 报错 ("There is no current event loop in thread ..."),
    所以 RPC 全部放主线程, pynput 监听线程只更新按键集合.
"""

import sys
import threading
import time

try:
    from pynput import keyboard
except ImportError:
    print("缺少 pynput, 安装: sudo apt install python3-pynput")
    sys.exit(1)

import cosysairsim as airsim


THROTTLE = 0.6     # 前进 / 后退 油门
STEER = 0.6        # 行进中转向幅度
SPIN_STEER = 1.0   # 原地自旋时左右轮差
TICK = 0.1         # 控制周期 (s), 10 Hz


class AGVTeleop:
    def __init__(self) -> None:
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.controls = airsim.CarControls()
        self.keys: set[str] = set()
        self.reset_pending = False
        self.lock = threading.Lock()
        self.running = True
        self._listener: keyboard.Listener | None = None
        print("[OK] CarClient (SkidVehicle) ready, API enabled")

    # ---- 仅在监听线程跑, 绝不调用 RPC ----
    def _key_to_str(self, key) -> str | None:
        if isinstance(key, keyboard.KeyCode) and key.char is not None:
            return key.char.lower()
        if key == keyboard.Key.space:
            return "space"
        if key == keyboard.Key.esc:
            return "esc"
        return None

    def on_press(self, key) -> None:
        k = self._key_to_str(key)
        if k is None:
            return
        if k == "esc":
            self.running = False
            if self._listener is not None:
                self._listener.stop()
            return
        with self.lock:
            if k == "r":
                self.reset_pending = True
            else:
                self.keys.add(k)

    def on_release(self, key) -> None:
        k = self._key_to_str(key)
        if k is None:
            return
        with self.lock:
            self.keys.discard(k)

    # ---- 主线程跑, 所有 RPC 在这里 ----
    def step(self) -> None:
        with self.lock:
            keys = set(self.keys)
            reset = self.reset_pending
            self.reset_pending = False

        if reset:
            print("[CMD] reset")
            self.client.reset()
            self.client.enableApiControl(True)
            return

        forward = "w" in keys
        backward = "s" in keys
        left = "a" in keys
        right = "d" in keys
        spin_l = "q" in keys
        spin_r = "e" in keys
        handbrake = "space" in keys

        throttle = (THROTTLE if forward else 0.0) - (THROTTLE if backward else 0.0)

        if spin_l or spin_r:
            throttle = 0.0
            steer = (SPIN_STEER if spin_r else 0.0) - (SPIN_STEER if spin_l else 0.0)
        else:
            steer = (STEER if right else 0.0) - (STEER if left else 0.0)

        self.controls.throttle = throttle
        self.controls.steering = steer
        self.controls.brake = 0.0
        self.controls.handbrake = handbrake
        self.controls.is_manual_gear = False
        self.controls.manual_gear = 0
        self.client.setCarControls(self.controls)

    def run(self) -> None:
        self._listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self._listener.start()
        try:
            while self.running:
                try:
                    self.step()
                except Exception as e:
                    print(f"[WARN] step 异常: {e}")
                time.sleep(TICK)
        finally:
            try:
                self.controls.throttle = 0.0
                self.controls.steering = 0.0
                self.controls.brake = 1.0
                self.client.setCarControls(self.controls)
                time.sleep(0.2)
                self.client.enableApiControl(False)
            except Exception:
                pass
            if self._listener is not None:
                self._listener.stop()
            print("[BYE]")


if __name__ == "__main__":
    print(__doc__)
    AGVTeleop().run()
