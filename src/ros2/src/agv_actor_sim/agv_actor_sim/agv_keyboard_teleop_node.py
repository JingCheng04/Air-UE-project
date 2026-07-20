"""Keyboard teleop for the pseudo AGV.

Publishes CarControls to <topic_prefix>/car_cmd.

Keys:
  W/S  forward/backward
  A/D  left/right yaw
  Space stop
  Esc  exit
"""

from __future__ import annotations

import sys
import threading

import rclpy
from airsim_interfaces.msg import CarControls
from rclpy.node import Node

try:
    from pynput import keyboard
except ImportError:
    keyboard = None


class AgvKeyboardTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('agv_keyboard_teleop_node')

        self.declare_parameter('topic_prefix', '/sim_ugv/airsim_node/UGV_1')
        self.declare_parameter('linear_speed', 4.0)   # m/s
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('rate', 20.0)

        prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        rate = max(1.0, float(self.get_parameter('rate').value))

        self.pub = self.create_publisher(CarControls, f'{prefix}/car_cmd', 10)
        self.keys: set[str] = set()
        self.lock = threading.Lock()

        if keyboard is None:
            self.get_logger().error('Missing pynput. Install python3-pynput in your environment.')
            raise RuntimeError('pynput not installed')

        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('Keyboard teleop ready: W/S forward/backward, A/D turn, Space stop, Esc exit')

    def _key_str(self, key) -> str | None:
        char = getattr(key, 'char', None)
        if char is not None:
            return str(char).lower()
        if key == keyboard.Key.space:
            return 'space'
        if key == keyboard.Key.esc:
            return 'esc'
        return None

    def _on_press(self, key) -> None:
        k = self._key_str(key)
        if k is None:
            return
        if k == 'esc':
            self.listener.stop()
            rclpy.shutdown()
            return
        with self.lock:
            self.keys.add(k)

    def _on_release(self, key) -> None:
        k = self._key_str(key)
        if k is None:
            return
        with self.lock:
            self.keys.discard(k)

    def _tick(self) -> None:
        # Read current key set and directly map it to target speed.
        with self.lock:
            keys = set(self.keys)

        throttle = 0.0
        steering = 0.0
        if 'w' in keys:
            throttle += 1.0
        if 's' in keys:
            throttle -= 1.0
        if 'a' in keys:
            steering += 1.0
        if 'd' in keys:
            steering -= 1.0
        if 'space' in keys:
            throttle = 0.0
            steering = 0.0

        cmd = CarControls()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.throttle = throttle
        cmd.steering = steering
        cmd.brake = 0.0
        cmd.handbrake = False
        cmd.manual = False
        cmd.manual_gear = 0
        cmd.gear_immediate = True
        self.pub.publish(cmd)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = AgvKeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.listener.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
