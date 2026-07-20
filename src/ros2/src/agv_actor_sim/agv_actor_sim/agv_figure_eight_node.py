"""伪 AGV 8 字巡航节点.

通过持续发布 ``car_cmd`` 让 ``agv_actor_node`` 以恒定线速度行驶, 按时间在
4 段半圆之间切换转向, 形成一个由 4 个半径相同的半圆拼成的 8 字形轨迹.

默认参数:
    target_speed   2.5 m/s
    radius         10.0 m

在这组参数下角速度恒为:
    w = v / r = 0.25 rad/s

单段半圆弧长 ``pi * r`` , 对应持续时间 ``pi * r / v``. 节点按
``[+1, -1, -1, +1]`` 的转向序列循环, 依次跑完 4 个半圆后回到起点附近, 再重复.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from airsim_interfaces.msg import CarControls
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class AgvFigureEightNode(Node):
    """以恒速发布 8 字轨迹控制指令."""

    def __init__(self) -> None:
        super().__init__("agv_figure_eight_node")

        self.declare_parameter("topic_prefix", "/sim_ugv/airsim_node/UGV_1")
        self.declare_parameter("target_speed", 2.5)
        self.declare_parameter("radius", 10.0)
        self.declare_parameter("max_speed", 2.5)
        self.declare_parameter("max_yaw_rate", 90.0)
        self.declare_parameter("rate", 30.0)

        prefix = self._param_str("topic_prefix").rstrip("/")
        self.target_speed = max(0.1, self._param_float("target_speed"))
        self.radius = max(0.1, self._param_float("radius"))
        self.max_speed = max(0.1, self._param_float("max_speed"))
        self.max_yaw_rate = math.radians(max(1.0, self._param_float("max_yaw_rate")))
        rate_hz = max(1.0, self._param_float("rate"))
        self.period = 1.0 / rate_hz

        self.phase_signs: Sequence[float] = (1.0, -1.0, -1.0, 1.0)
        self.phase_duration = math.pi * self.radius / self.target_speed
        self.elapsed_in_phase = 0.0
        self.phase_index = 0
        self.have_odom = False
        self._cmd_count = 0
        self._tick_count = 0

        cmd_topic = f"{prefix}/car_cmd"
        odom_topic = f"{prefix}/odom_local"
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_pub = self.create_publisher(CarControls, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, odom_qos)

        yaw_rate_cmd = self.target_speed / self.radius
        steering_cmd = yaw_rate_cmd / self.max_yaw_rate
        if abs(steering_cmd) > 1.0:
            self.get_logger().warn(
                f"figure-eight steering requires {steering_cmd:+.2f}, exceeds actuator limit; "
                "command will be clamped"
            )

        self.get_logger().info(
            f"publishing figure-eight on {cmd_topic}; waiting odom on {odom_topic}; "
            f"v={self.target_speed:.2f} m/s, radius={self.radius:.2f} m, "
            f"half_circle_time={self.phase_duration:.2f} s"
        )

        self.create_timer(self.period, self._tick)

    def _param_str(self, name: str) -> str:
        value = self.get_parameter(name).value
        return "" if value is None else str(value)

    def _param_float(self, name: str) -> float:
        value = self.get_parameter(name).value
        return float(0.0 if value is None else value)

    def _on_odom(self, _: Odometry) -> None:
        self.have_odom = True

    def _send(self, throttle: float, steering: float, brake: float = 0.0) -> None:
        msg = CarControls()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle = float(max(-1.0, min(1.0, throttle)))
        msg.steering = float(max(-1.0, min(1.0, steering)))
        msg.brake = float(max(0.0, min(1.0, brake)))
        msg.handbrake = False
        msg.manual = False
        msg.manual_gear = 0
        msg.gear_immediate = True
        self.cmd_pub.publish(msg)
        self._cmd_count += 1

    def _tick(self) -> None:
        self._tick_count += 1
        ticks_per_sec = max(1, int(round(1.0 / self.period)))
        if not self.have_odom:
            if self._tick_count % (ticks_per_sec * 2) == 1:
                self.get_logger().warn("waiting for odom_local before starting figure-eight")
            return

        yaw_rate_cmd = self.target_speed / self.radius
        throttle = self.target_speed / self.max_speed
        steering = self.phase_signs[self.phase_index] * yaw_rate_cmd / self.max_yaw_rate
        self._send(throttle, steering, brake=0.0)

        self.elapsed_in_phase += self.period
        if self.elapsed_in_phase >= self.phase_duration:
            self.elapsed_in_phase -= self.phase_duration
            self.phase_index = (self.phase_index + 1) % len(self.phase_signs)
            self.get_logger().info(
                f"switching to semicircle phase {self.phase_index} "
                f"(turn_sign={self.phase_signs[self.phase_index]:+.0f})"
            )

    def destroy_node(self) -> None:
        try:
            self._send(0.0, 0.0, brake=1.0)
        except Exception:
            pass
        super().destroy_node()


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = AgvFigureEightNode()
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
