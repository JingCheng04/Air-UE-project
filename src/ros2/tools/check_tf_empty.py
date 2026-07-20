#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFCheck(Node):
    def __init__(self):
        super().__init__("check_tf_empty")
        self.bad = 0
        self.total = 0
        self.create_subscription(TFMessage, "/tf", self.cb, 100)
        qos_static = QoSProfile(depth=100)
        qos_static.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_static.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(TFMessage, "/tf_static", self.cb, qos_static)

    def cb(self, msg):
        for t in msg.transforms:
            self.total += 1
            if not t.header.frame_id or not t.child_frame_id or t.header.frame_id == t.child_frame_id:
                self.bad += 1
                print("BAD_TF", repr(t.header.frame_id), repr(t.child_frame_id), "stamp", t.header.stamp.sec, t.header.stamp.nanosec)
            else:
                print("TF", repr(t.header.frame_id), "->", repr(t.child_frame_id))


def main():
    rclpy.init()
    node = TFCheck()
    end = time.time() + 5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"SUMMARY total={node.total} bad={node.bad}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
