"""伪 AGV 闭环驾驶测试节点 (前进 - 后退 - 原地转向).

行为与 src/test/agv/ros2_drive_test.py 一致, 但完全走 ROS2 接口:
    publish: <prefix>/car_cmd        airsim_interfaces/msg/CarControls
    subscribe: <prefix>/odom_local   nav_msgs/Odometry

依赖 agv_actor_node 把 car_cmd 转化为 actor 运动, 以及 agv_imu_odom_node
把 actor 位姿反推到 odom_local. 这两个节点必须先 launch 起来.

序列: 前进 --distance m -> 刹停 -> 后退到起点附近 -> 刹停 -> 原地转 --spin deg.
每个阶段前后打印一次位置和 yaw, 便于核对实际运动.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from airsim_interfaces.msg import CarControls
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AgvTestDrive(Node):
    """通过 ROS2 发 car_cmd 推动伪 AGV, 用 odom_local 做闭环反馈."""

    def __init__(self, prefix: str) -> None:
        super().__init__("agv_test_drive_node")
        self.cmd_pub = self.create_publisher(CarControls, f"{prefix}/car_cmd", 10)
        self.create_subscription(Odometry, f"{prefix}/odom_local", self._odom_cb, 10)

        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.get_logger().info(f"Subscribed:  {prefix}/odom_local")
        self.get_logger().info(f"Publishing:  {prefix}/car_cmd")

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def send(self, throttle: float, steering: float, brake: float = 0.0) -> None:
        cmd = CarControls()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.throttle = float(throttle)
        cmd.steering = float(steering)
        cmd.brake = float(brake)
        cmd.handbrake = False
        cmd.manual = False
        cmd.manual_gear = 0
        cmd.gear_immediate = True
        self.cmd_pub.publish(cmd)

    def wait_for_odom(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.have_odom:
                return True
        return False

    def log_pose(self, label: str) -> None:
        self.get_logger().info(
            f"[{label}] x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f}deg"
        )

    def run_until(self, predicate, throttle: float, steering: float, label: str,
                  timeout: float = 30.0, brake: float = 0.0) -> None:
        self.get_logger().info(f"[{label}] start")
        deadline = time.time() + timeout
        while rclpy.ok() and not predicate():
            if time.time() > deadline:
                self.get_logger().warn(f"[{label}] timed out")
                break
            self.send(throttle, steering, brake)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f"[{label}] done")

    def brake_for(self, seconds: float = 1.0) -> None:
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            self.send(0.0, 0.0, brake=1.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", default="/sim_ugv/airsim_node/UGV_1")
    parser.add_argument("--distance", type=float, default=3.0,
                        help="meters to move forward and back")
    parser.add_argument("--spin", type=float, default=90.0,
                        help="degrees to spin in place")
    args = parser.parse_args()

    rclpy.init()
    node = AgvTestDrive(args.prefix)

    if not node.wait_for_odom():
        node.get_logger().error("no odom received, are agv_actor_node and agv_imu_odom_node up?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    start_x, start_y = node.x, node.y
    start_yaw = node.yaw
    target_dist = float(args.distance)
    target_spin = math.radians(float(args.spin))

    def reached_forward() -> bool:
        return math.hypot(node.x - start_x, node.y - start_y) >= target_dist

    def back_at_start() -> bool:
        return math.hypot(node.x - start_x, node.y - start_y) <= 0.4

    accumulated = [0.0]
    last_yaw = [node.yaw]

    def reached_spin() -> bool:
        delta = wrap(node.yaw - last_yaw[0])
        accumulated[0] += abs(delta)
        last_yaw[0] = node.yaw
        return accumulated[0] >= target_spin

    try:
        node.log_pose("before forward")
        node.run_until(reached_forward, throttle=0.5, steering=0.0, label="forward")
        node.brake_for()
        node.log_pose("after forward")

        node.run_until(back_at_start, throttle=-0.4, steering=0.0, label="backward")
        node.brake_for()
        node.log_pose("after backward")

        node.run_until(reached_spin, throttle=0.0, steering=1.0, label="spin")
        node.brake_for()
        node.log_pose("after spin")
    except KeyboardInterrupt:
        pass
    finally:
        node.brake_for(0.5)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
