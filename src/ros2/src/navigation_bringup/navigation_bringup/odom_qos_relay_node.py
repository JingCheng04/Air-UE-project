"""里程计 QoS 中继节点。

AirSim 以 RELIABLE QoS 发布里程计，而 Nav2 的 OdomSmoother 以 BEST_EFFORT 订阅，
两者不兼容导致 DWB 收不到里程计。本节点订阅原始里程计并以 BEST_EFFORT 重新发布。
"""
from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomQosRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_qos_relay_node')
        self.declare_parameter('input_topic', '/uav/airsim_node/UAV_1/odom_local')
        self.declare_parameter('output_topic', '/uav/airsim_node/UAV_1/odom_relay')

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)

        # 两端均使用 BEST_EFFORT，以匹配 Nav2 的 OdomSmoother 订阅端。
        self.pub = self.create_publisher(Odometry, out_topic, qos_profile_sensor_data)
        self.create_subscription(Odometry, in_topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Odometry) -> None:
        self.pub.publish(msg)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = OdomQosRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
