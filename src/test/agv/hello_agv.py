"""
hello_agv.py — Cosys-AirSim AGV (CPHusky / SkidVehicle) 最小驾驶示例

依次演示: 前进、左转、右转、原地左旋、原地右旋、停车、后退、刹停。

要点:
    - SkidVehicle 仍然用 CarClient + setCarControls 接口,
      但是内部把 (throttle, steering) 解释成左右轮差速.
    - throttle = +1 双轮正转, -1 双轮反转, 不需要切倒挡.
    - steering = ±1 表示左右轮速差最大. throttle=0 时即原地自旋.
    - 已知 bug: 原地左旋会有少量前向漂移 (Chaos 物理 + raw YawInput 引起).

参考: https://cosys-lab.github.io/Cosys-AirSim/skid_steer_vehicle/
"""

import time
import cosysairsim as airsim


def main() -> None:
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print(f"[OK] API control enabled = {client.isApiControlEnabled()}")

    controls = airsim.CarControls()

    def apply(label: str, hold: float, **kw) -> None:
        for k, v in kw.items():
            setattr(controls, k, v)
        client.setCarControls(controls)
        print(f">> {label}  {kw}")
        time.sleep(hold)
        s = client.getCarState()
        p = s.kinematics_estimated.position
        print(f"   speed={s.speed:5.2f}  pos=({p.x_val:6.2f}, {p.y_val:6.2f})")

    try:
        apply("前进",       3.0, throttle=0.6, steering=0.0, brake=0.0)
        apply("前进 + 右弯", 3.0, throttle=0.6, steering=0.5)
        apply("前进 + 左弯", 3.0, throttle=0.6, steering=-0.5)
        apply("刹停",       1.5, throttle=0.0, steering=0.0, brake=1.0)

        apply("原地右旋",   3.0, throttle=0.0, steering=1.0, brake=0.0)
        apply("原地左旋",   3.0, throttle=0.0, steering=-1.0)
        apply("刹停",       1.5, throttle=0.0, steering=0.0, brake=1.0)

        # 差速车后退: throttle 给负, 不需要 manual_gear
        apply("后退",       3.0, throttle=-0.5, steering=0.0, brake=0.0)
        apply("最终刹停",   2.0, throttle=0.0, steering=0.0, brake=1.0)

    finally:
        client.reset()
        client.enableApiControl(False)
        print(">> done, reset & released API control")


if __name__ == "__main__":
    main()
