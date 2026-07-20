"""UAV<->AGV 绑定/解绑节点.

通过 simSetKinematics 把 UAV 的位置、姿态、线速度、角速度整体覆盖到 AGV 上,
仿真 "无穷大摩擦". 解绑后 UAV 立即交还给 AirSim 飞控.

仅在 recovery_state ∈ {on_agv, recover_ok}, 或 recovery_cmd == bind 时绑定.
不会在 AGV 之外的物体上误绑.
"""
from __future__ import annotations

import math

import cosysairsim as airsim  # type: ignore[import-not-found]
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
    cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
    return math.atan2(siny, cosy)


def yaw_to_quat(yaw: float):
    h = yaw * 0.5
    return airsim.Quaternionr(0.0, 0.0, math.sin(h), math.cos(h))


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


class UavFollowAgvNode(Node):
    BIND_STATES = {"on_agv", "recover_ok"}

    def __init__(self) -> None:
        super().__init__("uav_follow_agv_node")

        self.declare_parameter("uav_prefix", "/uav/airsim_node/UAV_1")
        self.declare_parameter("uav_name", "UAV_1")
        self.declare_parameter("agv_object", "UGV_Husky")
        self.declare_parameter("host_ip", "127.0.0.1")
        self.declare_parameter("host_port", 41451)
        self.declare_parameter("rate", 60.0)

        uav_prefix = str(self.get_parameter("uav_prefix").value).rstrip("/")
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.agv_object = str(self.get_parameter("agv_object").value)
        host_ip = str(self.get_parameter("host_ip").value)
        host_port = int(self.get_parameter("host_port").value)
        rate = float(self.get_parameter("rate").value)

        self.client = airsim.MultirotorClient(ip=host_ip, port=host_port)
        self.client.confirmConnection()

        self.state = "flying"   # 默认不绑定; 等 recovery_state 收到 on_agv 才绑
        self.cmd = "auto"
        self.following = False
        self.offset = airsim.Vector3r(0.0, 0.0, 0.0)
        self.dyaw = 0.0

        self.create_subscription(
            String, f"{uav_prefix}/recovery_state", self._on_state, 10
        )
        self.create_subscription(
            String, f"{uav_prefix}/recovery_cmd", self._on_cmd, 10
        )
        self.create_timer(1.0 / max(1.0, rate), self._tick)

        self.get_logger().info(
            f"follow ready: uav={self.uav_name}, "
            f"agv={self.agv_object}, rate={rate} Hz"
        )

    def _on_state(self, msg: String) -> None:
        self.state = msg.data.strip()

    def _on_cmd(self, msg: String) -> None:
        self.cmd = msg.data.strip()
        # 收到 release 时立即停止 follow, 不等下一个 _tick 判断.
        # 这消除了 ROS 话题延迟导致的 simSetKinematics 与 takeoff 打架震颤.
        if self.cmd == "release" and self.following:
            self.following = False
            self.get_logger().info("unbind (immediate on release cmd)")

    def _capture(self) -> None:
        """记录 UAV 相对 AGV 的车体系偏移和 yaw 差."""
        agv = self.client.simGetObjectPose(self.agv_object)
        uav = self.client.simGetVehiclePose(vehicle_name=self.uav_name)
        if not _finite(
            agv.position.x_val, agv.position.y_val, agv.position.z_val,
            agv.orientation.x_val, agv.orientation.y_val, agv.orientation.z_val, agv.orientation.w_val,
            uav.position.x_val, uav.position.y_val, uav.position.z_val,
            uav.orientation.x_val, uav.orientation.y_val, uav.orientation.z_val, uav.orientation.w_val,
        ):
            self.get_logger().warn("skip bind: AirSim pose has NaN")
            return
        agv_yaw = quat_to_yaw(agv.orientation)
        dx = uav.position.x_val - agv.position.x_val
        dy = uav.position.y_val - agv.position.y_val
        dz = uav.position.z_val - agv.position.z_val
        c, s = math.cos(-agv_yaw), math.sin(-agv_yaw)
        self.offset = airsim.Vector3r(dx * c - dy * s, dx * s + dy * c, dz)
        self.dyaw = quat_to_yaw(uav.orientation) - agv_yaw
        self.get_logger().info(
            f"bind: offset=({self.offset.x_val:.2f},"
            f"{self.offset.y_val:.2f},{self.offset.z_val:.2f})"
        )

    def _apply(self) -> None:
        """每帧把 UAV 位姿 + 速度都覆盖, 等价于刚性约束."""
        agv = self.client.simGetObjectPose(self.agv_object)
        if not _finite(
            agv.position.x_val, agv.position.y_val, agv.position.z_val,
            agv.orientation.x_val, agv.orientation.y_val, agv.orientation.z_val, agv.orientation.w_val,
            self.offset.x_val, self.offset.y_val, self.offset.z_val, self.dyaw,
        ):
            self.get_logger().warn("skip follow frame: NaN in AGV pose or stored offset")
            self.following = False
            return
        agv_yaw = quat_to_yaw(agv.orientation)
        c, s = math.cos(agv_yaw), math.sin(agv_yaw)
        wx = agv.position.x_val + self.offset.x_val * c - self.offset.y_val * s
        wy = agv.position.y_val + self.offset.x_val * s + self.offset.y_val * c
        wz = agv.position.z_val + self.offset.z_val
        if not _finite(wx, wy, wz, agv_yaw + self.dyaw):
            self.get_logger().warn("skip follow frame: computed UAV state has NaN")
            self.following = False
            return
        state = airsim.KinematicsState()
        state.position = airsim.Vector3r(wx, wy, wz)
        state.orientation = yaw_to_quat(agv_yaw + self.dyaw)
        state.linear_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
        state.angular_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
        state.linear_acceleration = airsim.Vector3r(0.0, 0.0, 0.0)
        state.angular_acceleration = airsim.Vector3r(0.0, 0.0, 0.0)
        self.client.simSetKinematics(state, True, self.uav_name)

    def _should_follow(self) -> bool:
        if self.cmd == "release":
            return False
        if self.cmd == "bind":
            return True
        return self.state in self.BIND_STATES

    def _tick(self) -> None:
        should = self._should_follow()
        if should and not self.following:
            self._capture()
            if _finite(self.offset.x_val, self.offset.y_val, self.offset.z_val, self.dyaw):
                self.following = True
        elif not should and self.following:
            self.following = False
            self.get_logger().info("unbind")
        if self.following:
            self._apply()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = UavFollowAgvNode()
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
