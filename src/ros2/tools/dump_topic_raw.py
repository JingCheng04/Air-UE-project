#!/usr/bin/env python3
"""Dump a ROS2 topic as text and hexadecimal bytes.

Default target is the suspected failing topic:
  /ugv/airsim_node/instance_segmentation_labels

Usage:
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  python3 tools/dump_topic_raw.py
  python3 tools/dump_topic_raw.py /some/topic package/msg/Type
"""

from __future__ import annotations

import sys
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_TOPIC = "/ugv/airsim_node/instance_segmentation_labels"
DEFAULT_TYPE = "airsim_interfaces/msg/InstanceSegmentationList"


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02x}" for b in data)


def dump_strings(obj: Any, path: str = "msg") -> None:
    if isinstance(obj, str):
        raw = obj.encode("utf-8", errors="surrogateescape")
        nul = "  <-- contains NUL" if b"\x00" in raw else ""
        print(f"[STRING] {path}: {obj!r}")
        print(f"[STRING_HEX] {path}: {hex_bytes(raw)}{nul}")
        return

    if isinstance(obj, (list, tuple)):
        for i, item in enumerate(obj):
            dump_strings(item, f"{path}[{i}]")
        return

    slots = getattr(obj, "__slots__", None)
    if slots:
        for slot in slots:
            name = slot[1:] if slot.startswith("_") else slot
            try:
                value = getattr(obj, name)
            except AttributeError:
                value = getattr(obj, slot)
            dump_strings(value, f"{path}.{name}")


class DumpNode(Node):
    def __init__(self, topic: str, type_name: str) -> None:
        super().__init__("dump_topic_raw")
        msg_type = get_message(type_name)
        self.sub = self.create_subscription(msg_type, topic, self.callback, 10)
        self.topic = topic
        self.type_name = type_name
        print(f"[INFO] Subscribed: {topic}")
        print(f"[INFO] Type: {type_name}")

    def callback(self, msg: Any) -> None:
        print("\n========== MESSAGE TEXT ==========")
        print(msg)

        print("========== STRING FIELDS ==========")
        dump_strings(msg)

        raw = serialize_message(msg)
        print("========== SERIALIZED CDR HEX ==========")
        print(hex_bytes(raw))
        print("========== END ==========")


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0

    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC
    type_name = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TYPE

    rclpy.init()
    node = DumpNode(topic, type_name)
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
