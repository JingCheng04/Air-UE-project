"""
fly_square.py — 让多旋翼按 10m x 10m 方形飞一圈

演示 moveToPositionAsync 的用法。NED 坐标系: +X 北, +Y 东, +Z 下。
"""

import time
import cosysairsim as airsim


SIDE = 10.0       # m, 方形边长
ALT = -5.0        # m, 巡航高度 (NED 下 z 为负即向上)
SPEED = 3.0       # m/s


def main() -> None:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    print(">> takeoff & climb")
    client.takeoffAsync().join()
    client.moveToZAsync(ALT, 2).join()

    waypoints = [
        (SIDE, 0.0),
        (SIDE, SIDE),
        (0.0, SIDE),
        (0.0, 0.0),
    ]
    for i, (x, y) in enumerate(waypoints, 1):
        print(f">> waypoint {i}/{len(waypoints)} -> ({x}, {y}, {ALT})")
        client.moveToPositionAsync(x, y, ALT, SPEED).join()
        time.sleep(0.5)

    print(">> land")
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print(">> done")


if __name__ == "__main__":
    main()
