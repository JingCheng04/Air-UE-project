"""
record_path.py — 录制 Husky 在 UE 中实际走过的轨迹, 输出 waypoints.json

用法 (开两个终端):
    终端 1: python3 src/test/agv/keyboard_agv.py     # 手动开车
    终端 2: python3 src/test/agv/record_path.py      # 旁观录轨

按 Ctrl+C 停止录制, 轨迹保存到 src/test/agv/waypoints.json.

要点:
    - 本脚本不调用 enableApiControl, 所以不会和 keyboard_agv.py 抢控制
    - 5 Hz 采样位置 (NED 系)
    - 自动做空间稀疏化: 距离上一个 waypoint < MIN_STEP 就跳过, 防止停车时
      堆出一堆同位置点
"""

import json
import math
import os
import signal
import sys
import time

import cosysairsim as airsim


SAMPLE_HZ = 5.0
MIN_STEP = 0.5  # m, 两个 waypoint 之间最小空间间隔
OUTPUT = os.path.join(os.path.dirname(__file__), "waypoints.json")


def main() -> None:
    client = airsim.CarClient()
    client.confirmConnection()
    print("[OK] 已连接 (旁观模式, 不接管控制)")
    print(f"[OK] 采样 {SAMPLE_HZ:.1f} Hz, 输出 -> {OUTPUT}")
    print("    现在去另一个终端开车, 按 Ctrl+C 结束录制\n")

    waypoints: list[dict] = []
    period = 1.0 / SAMPLE_HZ

    def on_sigint(_sig, _frm) -> None:
        save_and_exit(waypoints)

    signal.signal(signal.SIGINT, on_sigint)

    last_x, last_y = None, None
    t0 = time.time()
    while True:
        s = client.getCarState()
        p = s.kinematics_estimated.position
        q = s.kinematics_estimated.orientation
        x, y, z = p.x_val, p.y_val, p.z_val

        if last_x is None or math.hypot(x - last_x, y - last_y) >= MIN_STEP:
            waypoints.append({
                "t": round(time.time() - t0, 3),
                "x": round(x, 3),
                "y": round(y, 3),
                "z": round(z, 3),
                "yaw_quat": [q.w_val, q.x_val, q.y_val, q.z_val],
                "speed": round(s.speed, 3),
            })
            last_x, last_y = x, y
            print(f"  [{len(waypoints):4d}] x={x:7.2f} y={y:7.2f} z={z:5.2f} v={s.speed:4.2f}")

        time.sleep(period)


def save_and_exit(waypoints: list[dict]) -> None:
    print(f"\n[STOP] 共采集 {len(waypoints)} 个 waypoint")
    if not waypoints:
        print("       没有数据, 不写文件")
        sys.exit(0)
    payload = {
        "frame": "NED",
        "min_step_m": MIN_STEP,
        "sample_hz": SAMPLE_HZ,
        "waypoints": waypoints,
    }
    with open(OUTPUT, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[OK] 已写入 {OUTPUT}")
    sys.exit(0)


if __name__ == "__main__":
    main()
