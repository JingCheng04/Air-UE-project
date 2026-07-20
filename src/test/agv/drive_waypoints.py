"""
drive_waypoints.py — Husky 沿 record_path.py 录的轨迹行驶

读取 src/test/agv/waypoints.json, 用纯追踪式 P 控制器跟随.
策略复用 drive_square.py 的连续控制 (无尖角, 不停车).

可选参数:
    --file PATH     指定 waypoint 文件, 默认 waypoints.json
    --loop          闭环, 走完最后一个点回到第一个点继续
    --speed FLOAT   巡航油门, 默认 0.6
"""

import argparse
import json
import math
import os
import sys
import time

import cosysairsim as airsim


DEFAULT_FILE = os.path.join(os.path.dirname(__file__), "waypoints.json")

ARRIVE_RADIUS = 1.5
APPROACH_DIST = 3.0
APPROACH_THROTTLE = 0.35
STEER_K = 1.6
INIT_ALIGN_THRESH = math.radians(60)
INIT_ALIGN_DONE = math.radians(15)
SPIN_STEER = 0.9
TICK = 0.05


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
    cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
    return math.atan2(siny, cosy)


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def load_waypoints(path: str) -> list[tuple[float, float]]:
    with open(path) as f:
        data = json.load(f)
    pts = [(w["x"], w["y"]) for w in data["waypoints"]]
    if not pts:
        raise SystemExit(f"[FAIL] {path} 里没有 waypoint")
    print(f"[INFO] 加载 {len(pts)} 个 waypoint, frame={data.get('frame', 'NED')}")
    return pts


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default=DEFAULT_FILE)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--speed", type=float, default=0.6)
    args = ap.parse_args()

    waypoints = load_waypoints(args.file)
    cruise = args.speed

    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    controls = airsim.CarControls()
    controls.is_manual_gear = False
    controls.brake = 0.0

    def get_pose():
        s = client.getCarState()
        p = s.kinematics_estimated.position
        q = s.kinematics_estimated.orientation
        return p.x_val, p.y_val, yaw_from_quat(q)

    def send(throttle: float, steer: float) -> None:
        controls.throttle = max(-1.0, min(1.0, throttle))
        controls.steering = max(-1.0, min(1.0, steer))
        client.setCarControls(controls)

    try:
        loop_count = 0
        while True:
            for i, (tx, ty) in enumerate(waypoints, 1):
                # 启动对准 (仅当严重偏离时)
                x, y, yaw = get_pose()
                err = wrap(math.atan2(ty - y, tx - x) - yaw)
                if abs(err) > INIT_ALIGN_THRESH:
                    while abs(err) > INIT_ALIGN_DONE:
                        send(0.0, SPIN_STEER if err > 0 else -SPIN_STEER)
                        time.sleep(TICK)
                        x, y, yaw = get_pose()
                        err = wrap(math.atan2(ty - y, tx - x) - yaw)

                # 连续追踪
                while True:
                    x, y, yaw = get_pose()
                    dx, dy = tx - x, ty - y
                    dist = math.hypot(dx, dy)
                    if dist < ARRIVE_RADIUS:
                        if i % 10 == 1 or i == len(waypoints):
                            print(f"  [{i:4d}/{len(waypoints)}] reached, dist={dist:.2f}")
                        break

                    err = wrap(math.atan2(dy, dx) - yaw)
                    base = APPROACH_THROTTLE if dist < APPROACH_DIST else cruise
                    attenuation = max(0.3, math.cos(err) ** 2)
                    send(base * attenuation, STEER_K * err / math.pi)
                    time.sleep(TICK)

            loop_count += 1
            print(f">> 完成第 {loop_count} 圈")
            if not args.loop:
                break

        controls.throttle = 0.0
        controls.steering = 0.0
        controls.brake = 1.0
        client.setCarControls(controls)
        time.sleep(1.5)

    except KeyboardInterrupt:
        print("\n[STOP] 用户中断")
    finally:
        client.reset()
        client.enableApiControl(False)
        print(">> done")


if __name__ == "__main__":
    main()
