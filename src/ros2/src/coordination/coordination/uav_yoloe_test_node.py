"""YOLOE 识别测试节点.

最小化测试流程, 不依赖状态机, 直接通过 ``vel_cmd_body_frame`` 控制无人机:

1. WAIT_READY    - 等 odom 到位
2. RELEASE       - 发布 recovery_cmd=release, 解除 follow 节点的绑定
3. TAKEOFF       - 调用 AirSim takeoff 服务
4. ASCEND        - 爬升到相对起飞点 +10m
5. BACKUP        - 机体系沿 -x 后退 10m (世界系欧氏距离判定)
6. HOVER         - 停止下发速度, 仅保持高度环, 保持悬停以便观察识别

YOLOE 识别由配套的 ``yoloe_detector_node`` 节点负责发布以下话题
(本节点不直接做识别, 只发布无人机自身状态):
    /uav/yoloe/detections, /uav/yoloe/target_pose,
    /uav/yoloe/annotated_image, /uav/yoloe/detected

本节点额外发布:
    /uav/test/phase   std_msgs/String   当前阶段名 (1 Hz)
"""
from __future__ import annotations

import math
import time

import rclpy
from airsim_interfaces.msg import VelCmd
from airsim_interfaces.srv import Takeoff
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


# 输出话题: 与 uav_state_machine 一致的 body-frame 速度指令.
UAV_OUTPUT_CMD_TOPIC = "/uav/airsim_node/UAV_1/vel_cmd_body_frame"


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class UavYoloeTestNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_yoloe_test_node")

        # ---- 参数 ----
        self.declare_parameter("uav_prefix", "/uav/airsim_node/UAV_1")
        self.declare_parameter("uav_name", "UAV_1")
        self.declare_parameter("takeoff_height", 10.0)        # m
        self.declare_parameter("ascend_speed", 2.0)           # m/s
        self.declare_parameter("backup_distance", 10.0)       # m
        self.declare_parameter("backup_speed", 1.0)           # m/s
        self.declare_parameter("height_tolerance", 0.15)      # m, ASCEND 完成阈值

        self.uav_prefix = str(self.get_parameter("uav_prefix").value).rstrip("/")
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.takeoff_height = float(self.get_parameter("takeoff_height").value)
        self.ascend_speed = float(self.get_parameter("ascend_speed").value)
        self.backup_distance = float(self.get_parameter("backup_distance").value)
        self.backup_speed = float(self.get_parameter("backup_speed").value)
        self.height_tolerance = float(self.get_parameter("height_tolerance").value)

        # ---- 订阅: odom (BEST_EFFORT) ----
        self.create_subscription(
            Odometry, f"{self.uav_prefix}/odom_local",
            self._uav_cb, qos_profile_sensor_data,
        )

        # ---- 发布 ----
        self.cmd_pub = self.create_publisher(VelCmd, UAV_OUTPUT_CMD_TOPIC, 10)
        self.recovery_cmd_pub = self.create_publisher(
            String, f"{self.uav_prefix}/recovery_cmd", 10,
        )
        self.phase_pub = self.create_publisher(String, "/uav/test/phase", 10)

        # ---- 服务客户端 ----
        self.takeoff_cli = self.create_client(Takeoff, f"{self.uav_prefix}/takeoff")

        # ---- 状态 ----
        self.have_uav = False
        self.uav_x = self.uav_y = self.uav_z = 0.0
        # 起飞前 hover 位姿, ASCEND 用 (uav_z - hover_z) 作为相对高度.
        self.hover_z = 0.0
        # BACKUP 起点, 用世界系水平距离判定走够 10m.
        self._backup_x0 = 0.0
        self._backup_y0 = 0.0

        self.phase = "WAIT_READY"
        self._release_sent = False
        self._takeoff_requested = False
        self._takeoff_future = None
        self._takeoff_done = False

        # 主循环 20 Hz, 阶段日志 1 Hz.
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_phase)

        self.get_logger().info(
            f"yoloe test config: takeoff_height={self.takeoff_height:.1f}m, "
            f"backup_distance={self.backup_distance:.1f}m"
        )

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _uav_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.uav_x, self.uav_y, self.uav_z = p.x, p.y, p.z
        self.have_uav = True

    def _publish_phase(self) -> None:
        msg = String()
        msg.data = self.phase
        self.phase_pub.publish(msg)

    # ------------------------------------------------------------------
    # 控制工具
    # ------------------------------------------------------------------
    def _send_release(self) -> None:
        if self._release_sent:
            return
        msg = String()
        msg.data = "release"
        self.recovery_cmd_pub.publish(msg)
        self._release_sent = True
        self.get_logger().info("recovery_cmd: release")

    def _height_vz(self, target_rel_z: float) -> float:
        """高度环: 把 (uav_z - hover_z) 拉到 target_rel_z. AirSim NED, z<0 向上."""
        err = target_rel_z - (self.uav_z - self.hover_z)
        if abs(err) < 0.05:
            return 0.0
        return clamp(-1.5 * err, -1.0, 1.0)

    def _publish_zero_with_height_hold(self) -> None:
        cmd = VelCmd()
        cmd.twist.linear.z = self._height_vz(self.takeoff_height)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # 状态机
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self.phase == "WAIT_READY":
            if self.have_uav:
                self.phase = "RELEASE"
                self.get_logger().info("odom ready -> RELEASE")
            return

        if self.phase == "RELEASE":
            self._send_release()
            # 与文档建议一致: release 后等一两个 follow tick 再起飞 (~50ms 足够).
            self.phase = "RELEASE_WAIT"
            self._release_wait_until = time.time() + 0.1
            return

        if self.phase == "RELEASE_WAIT":
            if time.time() >= self._release_wait_until:
                self.phase = "TAKEOFF"
                self.get_logger().info("release wait done -> TAKEOFF")
            return

        if self.phase == "TAKEOFF":
            if not self._takeoff_requested:
                if not self.takeoff_cli.wait_for_service(timeout_sec=0.0):
                    return
                req = Takeoff.Request()
                req.wait_on_last_task = True
                self._takeoff_future = self.takeoff_cli.call_async(req)
                self._takeoff_requested = True
                self.get_logger().info("takeoff requested")
                return
            assert self._takeoff_future is not None
            if self._takeoff_future.done():
                self._takeoff_done = True
                # 用本次起飞后立即记录的 z 作为 0 高度参考.
                self.hover_z = self.uav_z
                self.phase = "ASCEND"
                self.get_logger().info(
                    f"takeoff done; hover_z={self.hover_z:.2f} -> ASCEND"
                )
            return

        if self.phase == "ASCEND":
            cmd = VelCmd()
            cmd.twist.linear.z = -self.ascend_speed  # NED: z<0 向上
            self.cmd_pub.publish(cmd)
            if (self.uav_z - self.hover_z) >= (self.takeoff_height - self.height_tolerance):
                self._backup_x0 = self.uav_x
                self._backup_y0 = self.uav_y
                self.phase = "BACKUP"
                self.get_logger().info(
                    f"reached {self.takeoff_height:.1f}m -> BACKUP"
                )
            return

        if self.phase == "BACKUP":
            traveled = math.hypot(
                self.uav_x - self._backup_x0,
                self.uav_y - self._backup_y0,
            )
            if traveled >= self.backup_distance:
                self.phase = "HOVER"
                self.get_logger().info(
                    f"backed {traveled:.2f}m -> HOVER (let YOLOE detect)"
                )
                # 立即发一帧 0 速 + 高度保持, 减小过冲.
                self._publish_zero_with_height_hold()
                return
            cmd = VelCmd()
            cmd.twist.linear.x = -self.backup_speed   # body x: 前为正, 后退取负
            cmd.twist.linear.y = 0.0
            cmd.twist.linear.z = self._height_vz(self.takeoff_height)
            self.cmd_pub.publish(cmd)
            return

        if self.phase == "HOVER":
            # 只锁高度, 水平归零. YOLOE 节点会持续发布检测/标注图像话题.
            self._publish_zero_with_height_hold()
            return


def main(args=None) -> int:
    rclpy.init(args=args)
    node = UavYoloeTestNode()
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
