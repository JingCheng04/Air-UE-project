"""
keyboard_drone.py — 用键盘控制 Cosys-AirSim 多旋翼

控制按键:
    起飞 / 降落:    T  /  L
    解锁 / 锁桨:    M  /  N        (Arm / Disarm)
    悬停:           H
    重置:           R
    退出:           Esc

    平移 (机体系, 持续按住):
        W / S       前进 / 后退  (+X / -X)
        A / D       左移 / 右移  (-Y / +Y)
        Space       上升        (-Z)
        Shift       下降        (+Z)

    偏航 (持续按住):
        Q / E       左转 / 右转

依赖:
    sudo apt install python3-pynput

实现说明:
    msgpack-rpc-python 在 Python 3.12 + 子线程里因缺 asyncio event loop
    会报 "There is no current event loop in thread ...", 所以 RPC 全部
    放主线程, pynput 监听只更新共享按键集合 + 一次性命令队列.
"""

import sys
import threading
import time
from collections import deque

try:
    from pynput import keyboard
except ImportError:
    print("缺少 pynput, 安装: sudo apt install python3-pynput")
    sys.exit(1)

import cosysairsim as airsim


LIN_SPEED = 3.0       # m/s
VERT_SPEED = 2.0      # m/s
YAW_RATE = 45.0       # deg/s
TICK = 0.1            # s
CMD_DURATION = 0.3    # s, > TICK


class DroneTeleop:
    def __init__(self) -> None:
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print("[OK] 已连接, API 控制已启用, 已解锁")

        self.keys: set[str] = set()
        self.commands: deque[str] = deque()  # 一次性命令: takeoff/land/hover/arm/disarm/reset
        self.lock = threading.Lock()
        self.running = True
        self._listener: keyboard.Listener | None = None

    def _key_to_str(self, key) -> str | None:
        if isinstance(key, keyboard.KeyCode) and key.char is not None:
            return key.char.lower()
        if key == keyboard.Key.space:
            return "space"
        if key in (keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r):
            return "shift"
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
            if k in ("t", "l", "h", "m", "n", "r"):
                self.commands.append(k)
            else:
                self.keys.add(k)

    def on_release(self, key) -> None:
        k = self._key_to_str(key)
        if k is None:
            return
        with self.lock:
            self.keys.discard(k)

    def step(self) -> None:
        # 1. 先消化一次性命令
        with self.lock:
            cmds = list(self.commands)
            self.commands.clear()
            keys = set(self.keys)

        for c in cmds:
            if c == "t":
                print("[CMD] takeoff"); self.client.takeoffAsync()
            elif c == "l":
                print("[CMD] land"); self.client.landAsync()
            elif c == "h":
                print("[CMD] hover"); self.client.hoverAsync()
            elif c == "m":
                print("[CMD] arm"); self.client.armDisarm(True)
            elif c == "n":
                print("[CMD] disarm"); self.client.armDisarm(False)
            elif c == "r":
                print("[CMD] reset")
                self.client.reset()
                self.client.enableApiControl(True)
                self.client.armDisarm(True)

        # 2. 速度指令
        vx = (LIN_SPEED if "w" in keys else 0.0) - (LIN_SPEED if "s" in keys else 0.0)
        vy = (LIN_SPEED if "d" in keys else 0.0) - (LIN_SPEED if "a" in keys else 0.0)
        vz = (VERT_SPEED if "shift" in keys else 0.0) - (VERT_SPEED if "space" in keys else 0.0)
        yr = (YAW_RATE if "e" in keys else 0.0) - (YAW_RATE if "q" in keys else 0.0)

        self.client.moveByVelocityBodyFrameAsync(
            vx=vx, vy=vy, vz=vz,
            duration=CMD_DURATION,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yr),
        )

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
                self.client.hoverAsync().join()
                self.client.armDisarm(False)
                self.client.enableApiControl(False)
            except Exception:
                pass
            if self._listener is not None:
                self._listener.stop()
            print("[BYE]")


if __name__ == "__main__":
    print(__doc__)
    DroneTeleop().run()
