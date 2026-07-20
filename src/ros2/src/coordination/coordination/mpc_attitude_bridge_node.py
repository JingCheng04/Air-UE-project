"""把 ``coni_mpc_controller`` 的 ``mavros_msgs/AttitudeTarget`` 输出
转发至 PX4 飞控 (通过 MAVROS ``/mavros/setpoint_raw/attitude``).

设计要点:

* coni-mpc 输出的 ``AttitudeTarget`` 已经是 MAVROS 标准格式, 本节点只做
  启停仲裁和安全保护, 不修改控制律或坐标系.
* 桥接默认是 "未启用" 的 -- 必须收到 ``bridge_enable=True`` 之后才会
  开始转发 AttitudeTarget 到 MAVROS. 这样可以在 MPC 接管前后做控制权切换.
* 失能或长时间收不到 coni-mpc 指令 (``cmd_timeout`` 秒) 时, 停止转发,
  让上游 state_machine 的 velocity setpoint 继续控制飞机.
* 启用期间持续以固定频率转发最新一帧 AttitudeTarget, 满足 PX4 Offboard
  对连续 setpoint 流的要求.
* 整个节点没有任何控制律, 只是 "仲裁 + 转发器". 接入失败、断流、数值
  异常时停止转发, 回退到 state_machine 的 velocity 控制.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from mavros_msgs.msg import AttitudeTarget
from rclpy.node import Node
from std_msgs.msg import Bool


class MpcAttitudeBridgeNode(Node):
    """订阅 coni-mpc AttitudeTarget + 启用信号, 转发到 MAVROS."""

    def __init__(self) -> None:
        super().__init__("mpc_attitude_bridge_node")

        # ---- 节点参数 ----
        # 控制循环频率, 应略高于 coni-mpc 的 control_rate (默认 20 Hz).
        # PX4 Offboard 要求连续 setpoint 流, 这里 30 Hz 留足裕量.
        self.declare_parameter("rate", 30.0)
        # 多久没收到 coni-mpc AttitudeTarget 视为失流. 失流期间停止转发,
        # 让 state_machine velocity 控制继续生效.
        self.declare_parameter("cmd_timeout", 0.5)
        # coni-mpc AttitudeTarget 输入话题 (与 coni-mpc launch 的 control_topic 对齐).
        self.declare_parameter("attitude_input_topic", "/uav/coni_mpc/attitude_target")
        # MAVROS AttitudeTarget 输出话题 (PX4 Offboard attitude setpoint 标准接口).
        self.declare_parameter("attitude_output_topic", "/mavros/setpoint_raw/attitude")
        # 启用 / 失能信号话题. 由协调器发布.
        self.declare_parameter("enable_topic", "/uav/coni_mpc/bridge_enable")

        rate_hz = max(1.0, float(self.get_parameter("rate").value))
        self.period = 1.0 / rate_hz
        self.cmd_timeout = max(self.period, float(self.get_parameter("cmd_timeout").value))
        input_topic = str(self.get_parameter("attitude_input_topic").value)
        output_topic = str(self.get_parameter("attitude_output_topic").value)
        enable_topic = str(self.get_parameter("enable_topic").value)

        # ---- 状态 ----
        self._lock = threading.Lock()
        self._latest_msg: Optional[AttitudeTarget] = None
        self._last_msg_time: float = 0.0
        self._enabled: bool = False

        # ---- ROS 接口 ----
        self.create_subscription(AttitudeTarget, input_topic, self._on_attitude, 10)
        self.create_subscription(Bool, enable_topic, self._on_enable, 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, output_topic, 10)

        self.get_logger().info(
            f"MPC attitude bridge ready: {input_topic} -> {output_topic}, "
            f"rate={rate_hz:.1f}Hz, cmd_timeout={self.cmd_timeout:.2f}s, "
            f"enable_topic={enable_topic}"
        )
        self.create_timer(self.period, self._tick)

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_attitude(self, msg: AttitudeTarget) -> None:
        """缓存最近一帧 coni-mpc AttitudeTarget; 不做任何数学修改."""
        with self._lock:
            self._latest_msg = msg
            self._last_msg_time = self._now()

    def _on_enable(self, msg: Bool) -> None:
        """根据 enable 信号切换桥接启停状态.

        启用 -> True : 开始转发 AttitudeTarget 到 MAVROS.
        启用 -> False: 停止转发, 回退到 state_machine velocity 控制.
        """
        want = bool(msg.data)
        if want != self._enabled:
            self._enabled = want
            state = "ENABLED" if want else "DISABLED"
            self.get_logger().info(f"MPC attitude bridge {state}")

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        """周期性转发最新 AttitudeTarget 到 MAVROS, 满足 PX4 Offboard 连续流要求."""
        if not self._enabled:
            return

        with self._lock:
            msg = self._latest_msg
            last_t = self._last_msg_time
        now = self._now()

        # 失流: 停止转发. 不主动发悬停 AttitudeTarget, 让 state_machine 的
        # velocity setpoint 继续控制飞机 (PX4 会在多路 setpoint 间自动切换).
        if msg is None or (now - last_t) > self.cmd_timeout:
            if msg is None:
                self.get_logger().warn(
                    "MPC bridge enabled but no AttitudeTarget received yet",
                    throttle_duration_sec=2.0,
                )
            else:
                self.get_logger().warn(
                    f"MPC AttitudeTarget stale (age={now - last_t:.2f}s), "
                    f"stop forwarding, fallback to velocity control",
                    throttle_duration_sec=1.0,
                )
            return

        # 数值有效性检查: 防 NaN / inf 直接透传给 PX4.
        if not self._is_valid(msg):
            self.get_logger().warn(
                "non-finite AttitudeTarget from coni-mpc, skip forwarding",
                throttle_duration_sec=1.0,
            )
            return

        # 转发到 MAVROS. 刷新 header.stamp 以满足 PX4 时间戳新鲜度要求.
        out = AttitudeTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.type_mask = msg.type_mask
        out.orientation = msg.orientation
        out.body_rate = msg.body_rate
        out.thrust = msg.thrust
        self.attitude_pub.publish(out)

        self.get_logger().info(
            f"forwarding: body_rate=({msg.body_rate.x:+.2f}, "
            f"{msg.body_rate.y:+.2f}, {msg.body_rate.z:+.2f}), "
            f"thrust={msg.thrust:.3f}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def _is_valid(self, msg: AttitudeTarget) -> bool:
        """检查 AttitudeTarget 数值有效性, 防 NaN/inf 透传给 PX4."""
        return (
            math.isfinite(msg.body_rate.x)
            and math.isfinite(msg.body_rate.y)
            and math.isfinite(msg.body_rate.z)
            and math.isfinite(msg.thrust)
            and msg.thrust >= 0.0
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MpcAttitudeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
