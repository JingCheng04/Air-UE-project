"""
hello_drone.py — 起飞 / 悬停 / 移动 / 降落 最小示例

按官方 Cosys-AirSim 文档:
    https://cosys-lab.github.io/Cosys-AirSim/apis/

NED 坐标系: +X 北, +Y 东, +Z 下 (向下为正, 所以高度是 -Z)。
"""

import time
import cosysairsim as airsim


def main() -> None:
    client = airsim.MultirotorClient()
    client.confirmConnection()

    print(">> 申请 API 控制 + 解锁")
    client.enableApiControl(True)
    client.armDisarm(True)

    print(">> 起飞 (takeoffAsync)")
    client.takeoffAsync().join()
    time.sleep(1)

    print(">> 上升到 5 米 (Z = -5)")
    client.moveToZAsync(z=-5, velocity=2).join()

    print(">> 向北 (前) 移动 10 米")
    client.moveToPositionAsync(x=10, y=0, z=-5, velocity=3).join()

    print(">> 悬停 2 秒")
    client.hoverAsync().join()
    time.sleep(2)

    state = client.getMultirotorState()
    p = state.kinematics_estimated.position
    print(f">> 当前 NED 位置: x={p.x_val:.2f}, y={p.y_val:.2f}, z={p.z_val:.2f}")

    print(">> 返回原点")
    client.moveToPositionAsync(x=0, y=0, z=-5, velocity=3).join()

    print(">> 降落 (landAsync)")
    client.landAsync().join()

    print(">> 锁桨 + 释放控制")
    client.armDisarm(False)
    client.enableApiControl(False)
    print(">> 完成")


if __name__ == "__main__":
    main()
