"""ROS2 flight smoke test for the Cosys-AirSim UAV wrapper.

Sequence:
  1. takeoff service
  2. ascend to ALT meters above the start point (negative z in NED)
  3. fly forward DISTANCE meters in body frame
  4. hover briefly
  5. fly back to the start position
  6. land service

Topics / services:
  publish:    <prefix>/vel_cmd_body_frame   airsim_interfaces/msg/VelCmd
              <prefix>/vel_cmd_world_frame  airsim_interfaces/msg/VelCmd
  subscribe:  <prefix>/odom_local           nav_msgs/msg/Odometry
  service:    <prefix>/takeoff              airsim_interfaces/srv/Takeoff
              <prefix>/land                 airsim_interfaces/srv/Land

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/Air-UE-project/src/ros2/install/setup.bash
  python3 ros2_fly_test.py
  python3 ros2_fly_test.py --prefix /uav/airsim_node/UAV_1 --alt 5.0 --distance 3.0
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from airsim_interfaces.msg import VelCmd
from airsim_interfaces.srv import Land, Takeoff
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


DEFAULT_PREFIX = "/uav/airsim_node/UAV_1"
ASCEND_SPEED = 1.5    # m/s, positive z goes up in ROS odom_local
HORIZ_SPEED = 2.0     # m/s, body x for forward
TICK = 0.05           # control period


class FlyTest(Node):
    def __init__(self, prefix: str) -> None:
        super().__init__("ros2_fly_test")
        self.body_pub = self.create_publisher(VelCmd, f"{prefix}/vel_cmd_body_frame", 10)
        self.world_pub = self.create_publisher(VelCmd, f"{prefix}/vel_cmd_world_frame", 10)
        self.takeoff_cli = self.create_client(Takeoff, f"{prefix}/takeoff")
        self.land_cli = self.create_client(Land, f"{prefix}/land")
        # Wrapper publishes odom_local with best-effort QoS, match it.
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, f"{prefix}/odom_local", self._odom_cb, odom_qos)

        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.get_logger().info(f"Subscribed: {prefix}/odom_local")
        self.get_logger().info(f"Publishing : {prefix}/vel_cmd_body_frame, {prefix}/vel_cmd_world_frame")

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.x = p.x
        self.y = p.y
        self.z = p.z
        self.have_odom = True

    def wait_for_odom(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.have_odom:
                return True
        return False

    def call_takeoff(self) -> bool:
        # AirSim takeoff service is blocking and may return False even when the
        # multirotor actually lifted off; treat completion of the call (or
        # timeout) as good enough and verify altitude via odom afterwards.
        if not self.takeoff_cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("takeoff service unavailable")
            return False
        req = Takeoff.Request()
        req.wait_on_last_task = True
        future = self.takeoff_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            self.get_logger().warn("takeoff service did not return in 20s, continuing anyway")
        elif future.result() is None:
            self.get_logger().warn("takeoff service returned None, continuing anyway")
        elif not future.result().success:
            self.get_logger().warn("takeoff service returned success=False, continuing anyway")
        return True

    def call_land(self) -> bool:
        if not self.land_cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("land service unavailable")
            return False
        req = Land.Request()
        req.wait_on_last_task = True
        future = self.land_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        return future.done() and future.result() is not None and future.result().success

    def send_body(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> None:
        cmd = VelCmd()
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.linear.z = float(vz)
        cmd.twist.angular.z = float(yaw_rate)
        self.body_pub.publish(cmd)

    def send_world(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> None:
        cmd = VelCmd()
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.linear.z = float(vz)
        cmd.twist.angular.z = float(yaw_rate)
        self.world_pub.publish(cmd)

    def log_pose(self, label: str) -> None:
        self.get_logger().info(f"[{label}] pos x={self.x:.2f} y={self.y:.2f} z={self.z:.2f}")

    def hold(self, seconds: float) -> None:
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            self.send_body(0.0, 0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=TICK)

    def ascend_to(self, target_z: float) -> None:
        # Close the loop on odom z instead of assuming a fixed ENU/NED sign convention.
        self.get_logger().info(f"[ascend] target z={target_z:.2f}")
        deadline = time.time() + 30.0
        next_log = time.time() + 1.0
        while rclpy.ok() and abs(self.z - target_z) > 0.3:
            if time.time() > deadline:
                self.get_logger().warn(f"[ascend] timed out at z={self.z:.2f}, target={target_z:.2f}")
                break
            if time.time() > next_log:
                self.get_logger().info(f"[ascend] z={self.z:.2f}")
                next_log = time.time() + 1.0

            # AirSim wrapper currently forwards vel_cmd_world_frame.z straight into
            # AirSim's NED z-down command without ENU<->NED conversion. Drive the
            # sign from odom feedback so the script still converges on target_z.
            vz_cmd = -ASCEND_SPEED if self.z < target_z else ASCEND_SPEED
            self.send_world(0.0, 0.0, vz_cmd, 0.0)
            rclpy.spin_once(self, timeout_sec=TICK)
        self.hold(0.5)

    def fly_forward_meters(self, meters: float, sign: int = 1) -> None:
        # Move along body x using velocity command. Use odom xy distance as feedback.
        self.get_logger().info(f"[forward] {meters:.2f} m, sign={sign}")
        sx, sy = self.x, self.y
        deadline = time.time() + 30.0
        next_log = time.time() + 1.0
        while rclpy.ok() and math.hypot(self.x - sx, self.y - sy) < meters:
            if time.time() > deadline:
                self.get_logger().warn(f"[forward] timed out at xy=({self.x:.2f},{self.y:.2f})")
                break
            if time.time() > next_log:
                self.get_logger().info(f"[forward] xy=({self.x:.2f},{self.y:.2f}) dist={math.hypot(self.x - sx, self.y - sy):.2f}")
                next_log = time.time() + 1.0
            self.send_body(sign * HORIZ_SPEED, 0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=TICK)
        self.hold(0.5)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", default=DEFAULT_PREFIX)
    parser.add_argument("--alt", type=float, default=5.0, help="meters above start (positive)")
    parser.add_argument("--distance", type=float, default=3.0, help="meters to fly forward and back")
    args = parser.parse_args()

    rclpy.init()
    node = FlyTest(args.prefix)
    if not node.wait_for_odom():
        node.get_logger().error("no odom received, is the wrapper up?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    distance = float(args.distance)

    try:
        node.log_pose("before takeoff")
        if not node.call_takeoff():
            node.get_logger().error("takeoff failed")
            return 2
        # Let wrapper's internal takeoff settle, then re-read current z so the
        # ascent target is relative to the actual hover altitude rather than
        # the pre-takeoff position.
        node.hold(1.5)
        node.log_pose("after takeoff")
        start_z = node.z
        target_z = start_z + float(args.alt)
        node.get_logger().info(f"after takeoff: start_z={start_z:.2f}, target_z={target_z:.2f}")
        node.ascend_to(target_z)
        node.log_pose("after ascend")
        node.hold(0.5)
        node.fly_forward_meters(distance, sign=+1)
        node.log_pose("after forward")
        node.hold(1.0)
        node.fly_forward_meters(distance, sign=-1)
        node.log_pose("after backward")
        node.hold(0.5)
        node.call_land()
        node.log_pose("after land")
    except KeyboardInterrupt:
        pass
    finally:
        node.hold(0.3)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
