"""UAV 回收状态发布节点.

直接通过 AirSim API 读取世界帧位姿和 UAV 速度, 判定状态后发布到
{uav_prefix}/recovery_state. 不使用 odom_local, 因为各车辆的 odom_local
以自身起飞点为原点, 跨车辆比较 z 坐标会得到错误结果.

发布:
    {uav_prefix}/recovery_state   std_msgs/String  (10 Hz)

状态值 (短字符串, ASCII, 全小写, 下划线分隔, ROS 安全):
    on_agv          UAV 静止于 AGV 上
    flying          UAV 在空中
    landing         UAV 正在下降
    recover_ok      刚降落到 AGV (瞬态, 下一帧并入 on_agv)
    recover_fail    降落到 AGV 之外 (锁定到下一次起飞)
"""
from __future__ import annotations

import math

import cosysairsim as airsim  # type: ignore[import-not-found]
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


class UavRecoveryStateNode(Node):

    def __init__(self) -> None:
        super().__init__("uav_recovery_state_node")

        self.declare_parameter("uav_prefix", "/uav/airsim_node/UAV_1")
        self.declare_parameter("uav_name", "UAV_1")
        self.declare_parameter("agv_object", "UGV_Husky")
        self.declare_parameter("host_ip", "127.0.0.1")
        self.declare_parameter("host_port", 41451)
        self.declare_parameter("vz_takeoff", 0.3)
        self.declare_parameter("vz_land", -0.2)
        self.declare_parameter("vz_settle", 0.2)
        self.declare_parameter("hold_frames", 5)
        self.declare_parameter("agv_xy_radius", 1.5)
        self.declare_parameter("agv_z_window", 2.0)

        uav_prefix = str(self.get_parameter("uav_prefix").value).rstrip("/")
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.agv_object = str(self.get_parameter("agv_object").value)
        host_ip = str(self.get_parameter("host_ip").value)
        host_port = int(self.get_parameter("host_port").value)
        self.vz_takeoff = float(self.get_parameter("vz_takeoff").value)
        self.vz_land = float(self.get_parameter("vz_land").value)
        self.vz_settle = float(self.get_parameter("vz_settle").value)
        self.hold_frames = int(self.get_parameter("hold_frames").value)
        self.agv_xy_radius = float(self.get_parameter("agv_xy_radius").value)
        self.agv_z_window = float(self.get_parameter("agv_z_window").value)

        self.client = airsim.MultirotorClient(ip=host_ip, port=host_port)
        self.client.confirmConnection()

        self.state_pub = self.create_publisher(
            String, f"{uav_prefix}/recovery_state", 10
        )

        self.state: str | None = None
        self._counters = {"up": 0, "down": 0, "settle": 0}

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"recovery_state -> {uav_prefix}/recovery_state, "
            f"uav={self.uav_name}, agv={self.agv_object}, "
            f"xy_radius={self.agv_xy_radius}m, z_window={self.agv_z_window}m"
        )

    def _read_world(self):
        agv = self.client.simGetObjectPose(self.agv_object)
        uav = self.client.simGetVehiclePose(vehicle_name=self.uav_name)
        kin = self.client.simGetGroundTruthKinematics(vehicle_name=self.uav_name)
        return (
            uav.position.x_val, uav.position.y_val, uav.position.z_val,
            agv.position.x_val, agv.position.y_val, agv.position.z_val,
            kin.linear_velocity.z_val,
        )

    def _over_agv(self, ux, uy, uz, ax, ay, az) -> bool:
        if math.hypot(ux - ax, uy - ay) > self.agv_xy_radius:
            return False
        if abs(uz - az) > self.agv_z_window:
            return False
        return True

    def _bump(self, key: str) -> None:
        for k in self._counters:
            self._counters[k] = self._counters[k] + 1 if k == key else 0

    def _publish(self, value: str) -> None:
        msg = String()
        msg.data = value
        self.state_pub.publish(msg)

    def _tick(self) -> None:
        ux, uy, uz, ax, ay, az, vz = self._read_world()
        if not _finite(ux, uy, uz, ax, ay, az, vz):
            # UAV/AGV 刚 spawn 或 RPC 尚未稳定时, AirSim 可能回 NaN. 直接跳过本帧.
            return
        # AirSim NED: vz>0 向下, vz<0 向上. 状态机里"向上"= 起飞, 反向一下.
        vz_up = -vz
        on = self._over_agv(ux, uy, uz, ax, ay, az)

        if self.state is None:
            self.state = "on_agv" if on else "flying"
            self.get_logger().info(
                f"state init -> {self.state}; "
                f"uav=({ux:.2f},{uy:.2f},{uz:.2f}), "
                f"agv=({ax:.2f},{ay:.2f},{az:.2f})"
            )

        if vz_up > self.vz_takeoff:
            self._bump("up")
        elif vz_up < self.vz_land:
            self._bump("down")
        elif abs(vz_up) < self.vz_settle:
            self._bump("settle")
        else:
            self._counters = {"up": 0, "down": 0, "settle": 0}

        prev = self.state
        if self.state == "on_agv":
            if self._counters["up"] >= self.hold_frames:
                self.state = "flying"
        elif self.state == "flying":
            if self._counters["down"] >= self.hold_frames:
                self.state = "landing"
            elif self._counters["settle"] >= self.hold_frames and on:
                # 起始误判或低速悬停在 AGV 正上方时回到 on_agv
                self.state = "on_agv"
        elif self.state == "landing":
            if self._counters["settle"] >= self.hold_frames:
                self.state = "recover_ok" if on else "recover_fail"
        elif self.state == "recover_ok":
            self.state = "on_agv"
        elif self.state == "recover_fail":
            if self._counters["up"] >= self.hold_frames:
                self.state = "flying"

        if self.state != prev:
            self.get_logger().info(f"state: {prev} -> {self.state}")
        self._publish(self.state)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = UavRecoveryStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
