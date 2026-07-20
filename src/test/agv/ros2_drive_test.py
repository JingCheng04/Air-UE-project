"""ROS2 driving smoke test for the Cosys-AirSim AGV wrapper.

Sequence:
  1. drive forward until odom advances DISTANCE_M meters
  2. brake to a stop
  3. drive backward to roughly the start position
  4. brake to a stop
  5. spin in place by SPIN_DEG degrees
  6. brake and exit

Topics:
  publish: <prefix>/car_cmd     airsim_interfaces/msg/CarControls
  subscribe: <prefix>/odom_local nav_msgs/msg/Odometry

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/Air-UE-project/src/ros2/install/setup.bash
  python3 ros2_drive_test.py
  python3 ros2_drive_test.py --prefix /ugv/airsim_node/AGV_1 --distance 3.0 --spin 90
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


DEFAULT_PREFIX = "/ugv/airsim_node/AGV_1"
THROTTLE_FWD = 0.5
THROTTLE_BACK = -0.4
SPIN_STEER = 1.0
TICK = 0.05  # seconds


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class DriveTest(Node):
    def __init__(self, prefix: str) -> None:
        super().__init__("ros2_drive_test")
        self.cmd_pub = self.create_publisher(CarControls, f"{prefix}/car_cmd", 10)
        self.create_subscription(Odometry, f"{prefix}/odom_local", self._odom_cb, 10)

        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.get_logger().info(f"Subscribed: {prefix}/odom_local")
        self.get_logger().info(f"Publishing : {prefix}/car_cmd")

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
        yaw_deg = math.degrees(self.yaw)
        self.get_logger().info(f"[{label}] pos x={self.x:.2f} y={self.y:.2f} yaw={yaw_deg:.1f}deg")

    def run_for(self, throttle: float, steering: float, predicate, label: str, brake: float = 0.0) -> None:
        self.get_logger().info(f"[{label}] start")
        while rclpy.ok() and not predicate():
            self.send(throttle, steering, brake)
            rclpy.spin_once(self, timeout_sec=TICK)
        self.get_logger().info(f"[{label}] reached")

    def brake(self, hold: float = 1.0) -> None:
        end = time.time() + hold
        while rclpy.ok() and time.time() < end:
            self.send(0.0, 0.0, brake=1.0)
            rclpy.spin_once(self, timeout_sec=TICK)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", default=DEFAULT_PREFIX)
    parser.add_argument("--distance", type=float, default=3.0, help="meters to move forward and back")
    parser.add_argument("--spin", type=float, default=90.0, help="degrees to spin in place")
    args = parser.parse_args()

    rclpy.init()
    node = DriveTest(args.prefix)
    if not node.wait_for_odom():
        node.get_logger().error("no odom received, is the wrapper up?")
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
        node.run_for(THROTTLE_FWD, 0.0, reached_forward, "forward")
        node.brake()
        node.log_pose("after forward")
        node.run_for(THROTTLE_BACK, 0.0, back_at_start, "backward")
        node.brake()
        node.log_pose("after backward")
        node.run_for(0.0, SPIN_STEER, reached_spin, "spin")
        node.brake()
        node.log_pose("after spin")
    except KeyboardInterrupt:
        pass
    finally:
        node.brake(0.5)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
