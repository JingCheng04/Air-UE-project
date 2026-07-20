"""
drive_square.py — AGV (Husky) 沿方形 waypoint 行驶 (平滑版)

控制策略 (避免顿挫):
    - 启动时: 如果初始航向偏差 > 60°, 先原地自旋粗对准一次
    - 巡航中: 永远不停车, 用连续 P 控制器:
          steering = clip(K_steer * 航向误差 / pi)
          throttle = base * cos(航向误差)^2  (偏差越大油门越软, 但不停)
    - 接近 waypoint 收油门, 进入半径直接切换到下一点

要点 (修复上一版的卡顿):
    - 不再边走边切回原地自旋, 控制信号是连续函数, 不出现 throttle = 0 的尖角
    - 单线程主循环, 所有 RPC 都在主线程, 避免 msgpack-rpc + 子线程 event loop 报错
"""

import math
import time

import cosysairsim as airsim


# 路径 (X 北, Y 东, m). Husky ~1 m/s, 10x10m 方形约 60s 一圈
WAYPOINTS = [
    (10.0, 0.0),
    (10.0, 10.0),
    (0.0, 10.0),
    (0.0, 0.0),
]
ARRIVE_RADIUS = 1.5
CRUISE_THROTTLE = 0.7
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


def main() -> None:
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    controls = airsim.CarControls()
    controls.is_manual_gear = False
    controls.manual_gear = 0
    controls.brake = 0.0

    print(f"[INFO] {len(WAYPOINTS)} waypoints, AGV = Husky")

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
        for i, (tx, ty) in enumerate(WAYPOINTS, 1):
            print(f">> waypoint {i}/{len(WAYPOINTS)} -> ({tx:.1f}, {ty:.1f})")

            # 启动对准: 仅当航向严重偏离时
            x, y, yaw = get_pose()
            err = wrap(math.atan2(ty - y, tx - x) - yaw)
            if abs(err) > INIT_ALIGN_THRESH:
                print(f"   initial align, err={math.degrees(err):.1f} deg")
                while abs(err) > INIT_ALIGN_DONE:
                    send(0.0, SPIN_STEER if err > 0 else -SPIN_STEER)
                    time.sleep(TICK)
                    x, y, yaw = get_pose()
                    err = wrap(math.atan2(ty - y, tx - x) - yaw)

            # 连续 P 控制, 不停车
            while True:
                x, y, yaw = get_pose()
                dx, dy = tx - x, ty - y
                dist = math.hypot(dx, dy)
                if dist < ARRIVE_RADIUS:
                    print(f"   reached, dist={dist:.2f}")
                    break

                err = wrap(math.atan2(dy, dx) - yaw)
                base = APPROACH_THROTTLE if dist < APPROACH_DIST else CRUISE_THROTTLE
                # 偏差越大油门越软, 但下限保留 30%, 避免完全停车导致顿挫
                attenuation = max(0.3, math.cos(err) ** 2)
                throttle = base * attenuation
                steer = STEER_K * err / math.pi

                send(throttle, steer)
                time.sleep(TICK)

        # 全部到达后刹停
        controls.throttle = 0.0
        controls.steering = 0.0
        controls.brake = 1.0
        client.setCarControls(controls)
        print(">> all waypoints done, braking")
        time.sleep(2.0)

    finally:
        client.reset()
        client.enableApiControl(False)
        print(">> done")


if __name__ == "__main__":
    main()
