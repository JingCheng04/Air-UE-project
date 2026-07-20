"""向 DWB 发送一条从当前位姿到最新目标点的直线路径。

始终在无人机机体坐标系（"UAV_1"）中规划：当前位姿恒为原点 (0,0)，目标点已表示在
机体坐标系下，从而避免依赖 AirSim 的非标准里程计坐标系。

目标生命周期：同一时刻只保持一个在途的 FollowPath 目标，仅当机体系目标发生明显
变化（偏航/目标移动）或上一目标结束/中止时才重发，避免频繁重发导致 DWB 无法完成
避障动作。
"""
from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action.client import ActionClient
from rclpy.node import Node


BASE_FRAME = 'UAV_1'


class DwbGoalPathNode(Node):
    def __init__(self) -> None:
        super().__init__('dwb_goal_path_node')
        self.declare_parameter('goal_topic', '/uav/dwb_goal_pose')
        self.declare_parameter('follow_path_action', '/follow_path')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        # 定时评估是否需要发送新目标的周期。
        self.declare_parameter('send_period', 1.0)
        # 重发阈值：机体系目标点位移/偏航变化超过阈值才重发。
        self.declare_parameter('resend_position_threshold', 2.0)
        self.declare_parameter('resend_yaw_threshold', 0.4)
        # 两次发送之间的最小间隔，防止频繁提交刷屏 controller_server。
        self.declare_parameter('min_goal_send_interval', 1.0)

        self.controller_id = str(self.get_parameter('controller_id').value)
        self.goal_checker_id = str(self.get_parameter('goal_checker_id').value)
        self.resend_pos_th = float(self.get_parameter('resend_position_threshold').value)
        self.resend_yaw_th = float(self.get_parameter('resend_yaw_threshold').value)
        self.min_goal_send_interval = float(self.get_parameter('min_goal_send_interval').value)

        self.goal_pose: PoseStamped | None = None
        # 上一次成功发送的机体系终点；None 表示当前无在途目标。
        self._last_sent: tuple[float, float, float] | None = None
        self._last_send_wall_time = 0.0
        # 在途目标句柄，用于观察其状态；目标执行期间不发送新目标。
        self._active_goal_handle = None
        self._send_in_flight = False  # send_goal_async 进行中时为 True。

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_cb,
            10,
        )
        self.client = ActionClient(
            self, FollowPath, str(self.get_parameter('follow_path_action').value)
        )
        self.create_timer(float(self.get_parameter('send_period').value), self._tick)

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.goal_pose = msg

    def _is_goal_active(self) -> bool:
        # 只要句柄存在就视为活动，直到结果回调将其清空。
        return self._active_goal_handle is not None

    def _needs_resend(self, gx: float, gy: float, yaw: float) -> bool:
        if self._last_sent is None:
            return True
        lx, ly, lyaw = self._last_sent
        dpos = math.hypot(gx - lx, gy - ly)
        dyaw = abs(math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)))
        return dpos >= self.resend_pos_th or dyaw >= self.resend_yaw_th

    def _tick(self) -> None:
        if self.goal_pose is None or not self.client.server_is_ready():
            return
        if self._send_in_flight:
            return

        now = time.time()
        if (now - self._last_send_wall_time) < self.min_goal_send_interval:
            return

        gx = self.goal_pose.pose.position.x
        gy = self.goal_pose.pose.position.y
        yaw = self._yaw_from_quat(self.goal_pose.pose.orientation)

        if self._is_goal_active() and not self._needs_resend(gx, gy, yaw):
            # 让 DWB 继续执行当前目标，此时重发只会导致其被中止。
            return

        path = self._build_body_frame_path(gx, gy)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        self._send_in_flight = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._last_sent = (gx, gy, yaw)
        self._last_send_wall_time = now

    def _build_body_frame_path(self, gx: float, gy: float) -> Path:
        # 在 BASE_FRAME 中构建从 (0,0) 到目标的密集路径，所有位姿共用同一 header，
        # 避免控制器因时间戳不匹配而丢弃。
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = BASE_FRAME

        dist = math.hypot(gx, gy)
        steps = max(2, min(50, int(dist / 1.0) + 1))

        poses = []
        for i in range(steps + 1):
            t = i / steps
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = gx * t
            pose.pose.position.y = gy * t
            pose.pose.position.z = 0.0
            pose.pose.orientation = self.goal_pose.pose.orientation if self.goal_pose else pose.pose.orientation
            poses.append(pose)
        path.poses = poses
        return path

    def _on_goal_response(self, future) -> None:
        self._send_in_flight = False
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - 记录 action client 的所有异常
            self.get_logger().warn(f'send_goal_async failed: {exc}')
            self._active_goal_handle = None
            self._last_sent = None
            return

        if not handle.accepted:
            self.get_logger().warn('FollowPath goal rejected by controller_server')
            self._active_goal_handle = None
            self._last_sent = None
            return

        self._active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, _future) -> None:
        # 无论结果如何（成功/中止/取消），都清空句柄，交由下次 _tick 决定是否发新目标。
        self._active_goal_handle = None


def main(args=None) -> int:
    rclpy.init(args=args)
    node = DwbGoalPathNode()
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
