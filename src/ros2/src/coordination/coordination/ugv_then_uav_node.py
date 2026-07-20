"""UGV/UAV 协调 demo.

本节点只负责任务编排:
1. UGV 前进并停车 (默认 ugv_distance=0 时跳过)
2. UAV 解绑定、起飞、爬升到 uav_height (默认 8m)
3. UAV 朝目标 GPS 直线飞行
4. 2D LiDAR 在 obstacle_distance 内检测到障碍时, 切换到 AVOID_OBSTACLE 状态
   并把目标转给 Nav2/DWB 局部规划; AVOID 时不再发 cruise vel
5. 距目标 target_tolerance 内切到 LANDING, 等速垂直下降到地面
6. 落地后进入 DONE, 持续下发零速

避障轨迹不在本节点计算. Nav2/DWB 在 navigation_bringup 中基于 2D scan 与
local_costmap 计算 /uav/dwb_cmd_vel, 本节点只把它转换成 AirSim VelCmd 候选指令,
交给 uav_state_machine_node 选择转发.
"""
from __future__ import annotations

import math
import time

import rclpy
from airsim_interfaces.msg import CarControls, VelCmd
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import CommandBool, CommandLong
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float32, String

from coordination.apf_escape import (
    ApfEscapeConfig,
    ApfEscapeFilter,
    compute_repulsive_xy,
)


EARTH_RADIUS_M = 6378137.0

# MAV_CMD_DO_SET_MODE + PX4 custom main mode constants.
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

STATE_TOPIC = "/uav/state"
GO_TO_TARGET_CMD_TOPIC = "/uav/control/go_to_target_cmd"
AVOID_OBSTACLE_CMD_TOPIC = "/uav/control/avoid_obstacle_cmd"
LANDING_CMD_TOPIC = "/uav/control/landing_cmd"
DWB_CMD_TOPIC = "/uav/dwb_cmd_vel"
UAV_DWB_GOAL_TOPIC = "/uav/dwb_goal_pose"
SCAN_TOPIC = "/uav/scan"

STATE_GO_TO_TARGET = "GO_TO_TARGET"
STATE_AVOID_OBSTACLE = "AVOID_OBSTACLE"
STATE_LANDING = "LANDING"


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class UgvThenUavNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_then_uav_node")

        self.declare_parameter("ugv_prefix", "/sim_ugv/airsim_node/UGV_1")
        self.declare_parameter("uav_prefix", "/uav/airsim_node/UAV_1")
        self.declare_parameter("uav_gps_topic", "/mavros/global_position/global")
        self.declare_parameter("uav_name", "UAV_1")
        self.declare_parameter("ugv_distance", 1.0)
        self.declare_parameter("ugv_throttle", 0.5)
        self.declare_parameter("ugv_start_delay", 5.0)
        self.declare_parameter("uav_height", 8.0)
        self.declare_parameter("uav_ascend_speed", 1.0)
        self.declare_parameter("target_latitude", 45.72029159954166)
        self.declare_parameter("target_longitude", -123.93306904260915)
        self.declare_parameter("target_tolerance", 1.0)
        self.declare_parameter("cruise_speed", 0.8)
        self.declare_parameter("landing_speed", 1.5)
        self.declare_parameter("landing_z_tolerance", 0.2)
        # Hysteresis: switch to AVOID when an obstacle gets within obstacle_distance,
        # and only return to GO_TO_TARGET once it has been clearly farther than
        # obstacle_clear_distance for obstacle_clear_hold seconds.
        # Entry threshold matches the APF range_of_sight=8 so the spatial
        # repulsion turns on at the same moment AVOID engages. Exit
        # threshold is intentionally much larger (11m) so a jittery scan
        # cannot keep flipping the state machine between AVOID and
        # GO_TO_TARGET several times per second.
        self.declare_parameter("obstacle_distance", 7.0)
        self.declare_parameter("obstacle_clear_distance", 9.0)
        self.declare_parameter("obstacle_clear_hold", 0.4)
        # AVOID->GO_TO_TARGET 转换后的 cooldown: cooldown 内不发 cruise,
        # 让 APF/DWB 把飞机继续推离最后一次的障碍, 避免 cruise 直接朝
        # 最终目标方向把飞机拉回墙边. 主要解决"绕开 -> 退出 AVOID ->
        # cruise 把飞机重新拽进侧墙 -> 又撞" 的反复循环.
        self.declare_parameter("avoid_cooldown", 1.5)
        self.declare_parameter("dwb_cmd_timeout", 0.5)

        self.ugv_prefix = str(self.get_parameter("ugv_prefix").value).rstrip("/")
        self.uav_prefix = str(self.get_parameter("uav_prefix").value).rstrip("/")
        self.uav_gps_topic = str(self.get_parameter("uav_gps_topic").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.ugv_distance = float(self.get_parameter("ugv_distance").value)
        self.ugv_throttle = float(self.get_parameter("ugv_throttle").value)
        self.ugv_start_delay = float(self.get_parameter("ugv_start_delay").value)
        self.uav_height = float(self.get_parameter("uav_height").value)
        self.uav_ascend_speed = float(self.get_parameter("uav_ascend_speed").value)
        self.target_lat = float(self.get_parameter("target_latitude").value)
        self.target_lon = float(self.get_parameter("target_longitude").value)
        self.target_tolerance = float(self.get_parameter("target_tolerance").value)
        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.landing_speed = float(self.get_parameter("landing_speed").value)
        self.landing_z_tolerance = float(self.get_parameter("landing_z_tolerance").value)
        self.obstacle_distance_limit = float(self.get_parameter("obstacle_distance").value)
        self.obstacle_clear_distance = float(self.get_parameter("obstacle_clear_distance").value)
        self.obstacle_clear_hold = float(self.get_parameter("obstacle_clear_hold").value)
        self.avoid_cooldown = float(self.get_parameter("avoid_cooldown").value)
        self.dwb_cmd_timeout = float(self.get_parameter("dwb_cmd_timeout").value)

        # AirSim / pointcloud_to_laserscan 的传感器话题通常是 BEST_EFFORT.
        # 这里必须用 sensor_data QoS, 否则虽然能看到 topic, 实际收不到消息.
        #
        # 统一坐标参考: 整个项目锚定到 MAVROS ENU (x=East, y=North, z=Up).
        # 这同时满足 coni-mpc 的隐含 ENU/FLU 假设, 避免 wrapper NWU 与 PX4
        # 内部 NED 之间的 90 度世界系错位.
        self.create_subscription(Odometry, f"{self.ugv_prefix}/odom_local", self._ugv_cb, qos_profile_sensor_data)
        # 旧实现 (订阅 wrapper 的 odom_local, 是 NWU 系, 与 ENU 差 90 度,
        # 是之前巡航方向出现 90/180 度系统偏差的根因之一):
        #     self.create_subscription(Odometry, f"{self.uav_prefix}/odom_local", self._uav_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._uav_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, self.uav_gps_topic, self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, DWB_CMD_TOPIC, self._dwb_cmd_cb, 10)

        self.ugv_cmd_pub = self.create_publisher(CarControls, f"{self.ugv_prefix}/car_cmd", 10)
        self.state_pub = self.create_publisher(String, STATE_TOPIC, 10)
        self.target_cmd_pub = self.create_publisher(VelCmd, GO_TO_TARGET_CMD_TOPIC, 10)
        self.avoid_cmd_pub = self.create_publisher(VelCmd, AVOID_OBSTACLE_CMD_TOPIC, 10)
        self.landing_cmd_pub = self.create_publisher(VelCmd, LANDING_CMD_TOPIC, 10)
        self.dwb_goal_pub = self.create_publisher(PoseStamped, UAV_DWB_GOAL_TOPIC, 10)
        self.height_pub = self.create_publisher(Float32, f"/{self.uav_name}/uav_height", 10)
        self.recovery_cmd_pub = self.create_publisher(String, f"{self.uav_prefix}/recovery_cmd", 10)
        # PX4 起飞不再走 AirSim wrapper 的 takeoffAsync 服务, 而是先切到
        # OFFBOARD 再解锁; 真正爬升仍沿用现有 UAV_ASCEND 阶段的速度指令.
        # 这里直接用 COMMAND_LONG(MAV_CMD_DO_SET_MODE), 避免某些 MAVROS/FCU
        # 组合在 SetMode(custom_mode="OFFBOARD") 路径上报 "Unsupported FCU".
        self.cmd_long_cli = self.create_client(CommandLong, "/mavros/cmd/command")
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        self.have_ugv = self.have_uav = self.have_gps = False
        self.ugv_x = self.ugv_y = self.uav_x = self.uav_y = self.uav_z = 0.0
        self.uav_yaw = 0.0
        self.uav_lat = self.uav_lon = 0.0
        self.hover_z = 0.0
        self.ugv_start_x = self.ugv_start_y = 0.0
        self.min_scan_range = math.inf
        self.ready_time = None
        self.takeoff_requested = False
        self.takeoff_done = False
        self.offboard_requested = False
        self.offboard_future = None
        self.arm_requested = False
        self.arm_future = None
        self.offboard_retry_deadline = 0.0
        self.arm_retry_deadline = 0.0
        self.recovery_released = False
        self.state = "WAIT_READY"
        self.avoid_active = False
        self.avoid_clear_since: float | None = None
        # AVOID 退出时刻; 在 (avoid_exit_time + avoid_cooldown) 之前不发 cruise,
        # 让 APF/DWB 把飞机继续推离最后一次的障碍, 避免被 cruise 立刻拽回.
        # None 表示没在 cooldown 中.
        self.avoid_exit_time: float | None = None
        self.last_dwb_cmd_time = 0.0

        # 时变反向人工势场, 仅在 AVOID 期间且无人机近似停滞时启用. xy 平面.
        # 不会改变高度, 不会改变 AVOID/GO_TO_TARGET 的进入或退出条件,
        # 仅在 AVOID 状态下叠加在 DWB 输出之上, 帮助逃出局部最优.
        self.apf_config = ApfEscapeConfig()
        self.apf_filter = ApfEscapeFilter(self.apf_config)
        # 最近一次 LaserScan 的关键字段, 供 APF 在 _dwb_cmd_cb 里复用.
        # 用 inf 占位避免在拿到第一帧 scan 之前误触发.
        self._scan_ranges: list[float] = []
        self._scan_angle_min = 0.0
        self._scan_angle_increment = 0.0

        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._publish_height)
        self.get_logger().info(
            f"target=({self.target_lat:.8f}, {self.target_lon:.8f}), "
            f"ugv_distance={self.ugv_distance:.2f}m, obstacle_distance={self.obstacle_distance_limit:.2f}m, "
            f"uav_gps={self.uav_gps_topic}"
        )

    def _ugv_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.ugv_x, self.ugv_y = p.x, p.y
        self.have_ugv = True

    def _uav_cb(self, msg: Odometry) -> None:
        self.uav_x = msg.pose.pose.position.x
        self.uav_y = msg.pose.pose.position.y
        self.uav_z = msg.pose.pose.position.z
        self.uav_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_uav = True
        # APF 滑动窗口必须在所有飞行阶段持续累积位姿, 而不是仅在 AVOID 时.
        # 否则一旦从 AVOID 退回 GO_TO_TARGET, 窗口会被高速巡航期间的大位移塞满,
        # 下次再靠近障碍时 APF 在 stuck_distance 阈值下永远不会被认为"卡住",
        # 错过它本应介入的几秒.
        self.apf_filter.update_pose(time.time(), self.uav_x, self.uav_y)

    def _gps_cb(self, msg: NavSatFix) -> None:
        self.uav_lat, self.uav_lon = msg.latitude, msg.longitude
        self.have_gps = True

    def _scan_cb(self, msg: LaserScan) -> None:
        # 2D LiDAR 的全部有效距离都参与最近障碍判断.
        # 对于多方向都可移动的 UAV, 不能只看前方扇区, 否则会漏掉
        # 侧后方建筑, 导致错误切回 GO_TO_TARGET 或在绕障时撞到另一角.
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.min_scan_range = min(valid) if valid else math.inf

        # 缓存当前 scan 的几何参数, 让 _dwb_cmd_cb 中的 APF 计算复用.
        # 注意: 这里直接保留原始 ranges, 不做范围裁剪, 让 apf_escape 模块自己
        # 决定 range_of_sight 截断和 inf 过滤.
        self._scan_ranges = list(msg.ranges)
        self._scan_angle_min = float(msg.angle_min)
        self._scan_angle_increment = float(msg.angle_increment)

    def _dwb_cmd_cb(self, msg: Twist) -> None:
        # DWB 只用作 XY 平面避障. 把 angular 部分丢掉, 避免 UAV 因 angular.z
        # 持续偏航, 导致机体系下的目标方向漂走.
        lin_x = msg.linear.x
        lin_y = msg.linear.y

        # 时变反向 APF 叠加 / 接管.
        # 触发条件 (全部由 ApfEscapeFilter 内部判断):
        #   - 当前处于 AVOID 状态 (由本节点上层逻辑决定)
        #   - 滑动时间窗口内的 world 位移 < stuck_distance
        #   - 视距内存在至少一个开阔方向 (>= free_distance)
        # 满足时, APF 输出一个机体系 xy 速度增量.
        # 严格只动 xy, 不影响 linear.z, 也不影响 z 方向状态机.
        #
        # 接管模式: 当 ApfEscapeFilter 判定 UAV 已经卡住 (is_stuck_now()),
        # 完全使用 APF 输出而不是叠加在 DWB 上. 这是因为 DWB 在凹角里通常
        # 已经放弃 (Failed to make progress -> abort -> retry), 它的 cmd
        # 在反复打架, 跟着叠加只会让飞机抖. 卡死状态下用 APF 单独驱动,
        # 能形成方向稳定的强反向推力把飞机从死角拉出.
        if self.avoid_active and self._scan_ranges:
            now = time.time()
            # 注意: 位姿在 _uav_cb 已经持续累积进 apf_filter, 这里不再重复 update_pose,
            # 否则同一 tick 多次 update_pose 会让滑动窗口偏向高频帧, 失去物理含义.
            fx, fy, max_clear = compute_repulsive_xy(
                self._scan_ranges,
                self._scan_angle_min,
                self._scan_angle_increment,
                self.apf_config,
            )
            apf_vx, apf_vy = self.apf_filter.compute_escape_velocity(
                now, fx, fy, max_clear,
            )
            if self.apf_filter.is_stuck_now():
                # 接管: APF 完全替代 DWB 输出.
                lin_x = apf_vx
                lin_y = apf_vy
            else:
                # 正常: APF 作为时间惩罚辅助叠加在 DWB 上.
                lin_x += apf_vx
                lin_y += apf_vy

        # 距离-限速安全网: 不论 DWB / APF 输出多大, 当障碍非常近时按
        # "撞墙前能刹住" 的物理约束二次夹一刀.
        # vmax = sqrt(2 * decel * (d_min - safety))
        # decel = 5 m/s^2 (与 DWB decel_lim 一致), safety = 1.1m (机体半径
        # 0.6 + 0.5 余量, 1.1m 内 vmax=0 强制刹停, 防止侧墙剐蹭).
        # gate: 只在 min_scan < 4m 时启用. 中远距离 (>4m) 完全不限速,
        # DWB / APF 自己控制速度.
        # is_stuck_now 状态下不限速, 让 APF 全力把飞机拽出来.
        if (
            self.avoid_active
            and self._scan_ranges
            and not self.apf_filter.is_stuck_now()
        ):
            d_min = self.min_scan_range
            if math.isfinite(d_min) and d_min < 4.0:
                margin = max(d_min - 1.1, 0.05)
                vmax = math.sqrt(2.0 * 5.0 * margin)
                speed = math.hypot(lin_x, lin_y)
                if speed > vmax and speed > 1e-6:
                    scale = vmax / speed
                    lin_x *= scale
                    lin_y *= scale

        cmd = VelCmd()
        cmd.twist.linear.x = lin_x
        cmd.twist.linear.y = lin_y
        # 高度保持: AVOID 期间也运行高度环, 而不是发送 z=0.
        # 单纯 z=0 在长时间横向机动里会让 UAV 因为 roll/pitch 倾斜逐步抬高
        # (AirSim 多旋翼的耦合) 或下沉. 用同一个 _height_vz P 控制器主动锁住
        # hover_z + uav_height, 这样任何避障/逃逸动作都不会改变高度.
        # 起飞前 takeoff_done=False 时 _height_vz 也会输出合理值, 但此时不会
        # 触发 AVOID, 因此这条分支不会被走到, 安全.
        cmd.twist.linear.z = self._height_vz()
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.last_dwb_cmd_time = time.time()
        self.avoid_cmd_pub.publish(cmd)

    def _publish_height(self) -> None:
        msg = Float32()
        msg.data = float(self.uav_z - self.hover_z) if self.takeoff_done else 0.0
        self.height_pub.publish(msg)

    def _request_px4_takeoff_start(self) -> bool:
        # OFFBOARD + ARM replaces the old AirSim takeoff service. Once armed,
        # the existing UAV_ASCEND state sends upward commands through the state
        # machine and MAVROS.
        now = time.time()

        if not self.offboard_requested:
            if now < self.offboard_retry_deadline:
                return False
            if not self.cmd_long_cli.wait_for_service(timeout_sec=0.0):
                return False
            req = CommandLong.Request()
            req.broadcast = False
            req.command = MAV_CMD_DO_SET_MODE
            req.confirmation = 0
            req.param1 = float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            req.param2 = float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            self.offboard_future = self.cmd_long_cli.call_async(req)
            self.offboard_requested = True
            self.get_logger().info("PX4 OFFBOARD requested")
            return False

        if self.offboard_future is not None and not self.offboard_future.done():
            return False

        if self.offboard_future is not None:
            resp = self.offboard_future.result()
            if resp is None or not resp.success:
                # Request reached MAVROS/PX4 but was not accepted. Stay in
                # UAV_TAKEOFF and retry slowly instead of spamming every tick.
                self.get_logger().warn(
                    "PX4 OFFBOARD request was not accepted; retrying",
                    throttle_duration_sec=2.0,
                )
                self.offboard_requested = False
                self.offboard_future = None
                self.offboard_retry_deadline = now + 1.0
                self.arm_requested = False
                self.arm_future = None
                self.arm_retry_deadline = 0.0
                return False

        if not self.arm_requested:
            if now < self.arm_retry_deadline:
                return False
            if not self.arm_cli.wait_for_service(timeout_sec=0.0):
                return False
            req = CommandBool.Request()
            req.value = True
            self.arm_future = self.arm_cli.call_async(req)
            self.arm_requested = True
            self.get_logger().info("PX4 arm requested")
            return False

        if self.arm_future is None or not self.arm_future.done():
            return False

        arm_resp = self.arm_future.result()
        if arm_resp is None or not arm_resp.success:
            self.get_logger().warn(
                "PX4 arm request failed or timed out; retrying",
                throttle_duration_sec=2.0,
            )
            self.arm_requested = False
            self.arm_future = None
            self.arm_retry_deadline = now + 1.0
            return False

        return True

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _send_ugv(self, throttle: float, brake: float = 0.0) -> None:
        cmd = CarControls()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.throttle = throttle
        cmd.brake = brake
        cmd.gear_immediate = True
        self.ugv_cmd_pub.publish(cmd)

    def _release_uav(self) -> None:
        if self.recovery_released:
            return
        msg = String()
        msg.data = "release"
        self.recovery_cmd_pub.publish(msg)
        self.recovery_released = True

    def _target_delta_m(self) -> tuple[float, float]:
        # 由 GPS 经纬度差计算到目标的局部位移. 整套项目锚定到 MAVROS ENU,
        # 所以这里输出 (east, north), x=East, y=North, 与 /mavros/local_position
        # 的 ENU 习惯一致, 也就和 coni-mpc 的 ENU/FLU 假设一致.
        lat = math.radians(self.uav_lat)
        north = math.radians(self.target_lat - self.uav_lat) * EARTH_RADIUS_M
        east = math.radians(self.target_lon - self.uav_lon) * EARTH_RADIUS_M * math.cos(lat)
        # 旧实现 (NED 习惯, 返回 (north, east)):
        #     return north, east
        return east, north

    def _height_vz(self) -> float:
        # 整套项目锚定到 MAVROS ENU: z 轴向上, 上升对应 vz>0.
        # err = 期望高度 - 当前高度. 当前飞机偏低 (err>0) 时需要上升, 输出 vz>0.
        err = self.uav_height - (self.uav_z - self.hover_z)
        if abs(err) < 0.05:
            return 0.0

        return clamp(1.5 * err, -1.0, 1.0)

    def _publish_dwb_goal_only(self) -> tuple[float, float, float]:
        # 计算并发布机体系 goal, 返回 (x_body, y_body, dist) 给上层决定是否还
        # 要发 cruise vel.
        #
        # 坐标系一致性 (整体一套, 锚定 MAVROS ENU):
        # - _target_delta_m() 输出 (east, north), 即 ENU 世界系下的目标位移.
        # - 订阅的 odom 来源切换到了 /mavros/local_position/odom, 它的世界系是
        #   ENU, 机体系是 FLU, yaw=0 朝东, +yaw 逆时针.
        # - 所以投影使用标准 ENU yaw 公式即可, 不再做 NWU 修正.
        # - 输出语义: x_body 朝机头前方 (FLU forward), y_body 朝机体左侧
        #   (FLU left). 下游 vel_cmd_to_attitude_target 按 ENU/FLU 解释.
        east, north = self._target_delta_m()
        dist = math.hypot(east, north)

        c = math.cos(self.uav_yaw)
        s = math.sin(self.uav_yaw)
        # ENU -> FLU: forward = east*cos(yaw) + north*sin(yaw)
        #             left    = -east*sin(yaw) + north*cos(yaw)
        x_body = east * c + north * s
        y_body = -east * s + north * c

        # 旧实现 (基于 wrapper NWU odom + NED GPS, 会引入 90 度世界系错位):
        #     north, east = self._target_delta_m()
        #     nwu_x = north
        #     nwu_y = -east
        #     forward = nwu_x * c + nwu_y * s
        #     leftward = -nwu_x * s + nwu_y * c
        #     x_body = forward
        #     y_body = -leftward

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.uav_name  # body frame == "UAV_1"
        goal.pose.position.x = x_body
        goal.pose.position.y = y_body
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.dwb_goal_pub.publish(goal)
        return x_body, y_body, dist

    def _publish_target_cmd(self) -> float:
        x_body, y_body, dist = self._publish_dwb_goal_only()

        cmd = VelCmd()
        if dist > 1e-3:
            speed = min(self.cruise_speed, 0.8 * dist)
            cmd.twist.linear.x = speed * x_body / dist
            cmd.twist.linear.y = speed * y_body / dist
        cmd.twist.linear.z = self._height_vz()
        self.target_cmd_pub.publish(cmd)
        return dist

    def _publish_landing_cmd(self) -> float:
        # 垂直降落: 水平速度归零, 直接以 landing_speed 等速下降, 不做高度环.
        # 整套项目锚定 MAVROS ENU/FLU: z 轴向上, 下降对应 vz<0.
        # 落地判定仍由 odom 世界系 z (ENU, 向上为正) 给出, 用 landing_z_tolerance
        # 决定何时切到 DONE 停止下发指令.
        cmd = VelCmd()
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = -self.landing_speed
        # 旧实现 (NED, 向下为正, 下降时 vz>0):
        #     cmd.twist.linear.z = self.landing_speed
        self.landing_cmd_pub.publish(cmd)
        return abs(self.uav_z)

    def _publish_zero_cmd(self) -> None:
        cmd = VelCmd()
        self.landing_cmd_pub.publish(cmd)

    def _update_obstacle_avoidance_state(self) -> None:
        """更新避障迟滞状态机 (进入容易, 退出需清场+cooldown).
        
        由子类和父类的巡航阶段调用, 确保避障逻辑一致. 只负责更新
        self.avoid_active / self.avoid_clear_since / self.avoid_exit_time,
        不发布任何指令或状态.
        """
        now = time.time()
        obstacle_seen = self.min_scan_range <= self.obstacle_distance_limit

        # 避障迟滞: 进入容易, 退出要求障碍清场并稳定一段时间, 防止 scan 抖动
        # 反复触发. 退出后还要再保持 avoid_cooldown 秒不发 cruise (avoid_active
        # 仍为 True), 让 APF/DWB 继续把飞机推离最后那一面墙, 避免 cruise 朝
        # 最终目标方向把飞机直接拉回墙边.
        if not self.avoid_active:
            if obstacle_seen:
                self.avoid_active = True
                self.avoid_clear_since = None
                self.avoid_exit_time = None
        else:
            if self.avoid_exit_time is not None:
                # 已经在 cooldown: 障碍重新进入触发距离就直接撤销 cooldown,
                # 重新视为正经避障; 否则到时间就退出.
                if obstacle_seen:
                    self.avoid_exit_time = None
                    self.avoid_clear_since = None
                elif (now - self.avoid_exit_time) >= self.avoid_cooldown:
                    self.avoid_active = False
                    self.avoid_exit_time = None
            elif self.min_scan_range >= self.obstacle_clear_distance:
                if self.avoid_clear_since is None:
                    self.avoid_clear_since = now
                elif (now - self.avoid_clear_since) >= self.obstacle_clear_hold:
                    # 进入 cooldown: 仍在 AVOID, 但开始倒计时.
                    self.avoid_exit_time = now
                    self.avoid_clear_since = None
            else:
                self.avoid_clear_since = None

    def _tick(self) -> None:
        if self.state == "WAIT_READY":
            if self.have_ugv and self.have_uav and self.have_gps:
                self.ugv_start_x, self.ugv_start_y = self.ugv_x, self.ugv_y
                self.ready_time = time.time()
                self.state = "WAIT_START_DELAY"
                self.get_logger().info("inputs ready")
            return

        if self.state == "WAIT_START_DELAY":
            if time.time() - self.ready_time >= self.ugv_start_delay:
                self.state = "UGV_FORWARD" if self.ugv_distance > 0.0 else "UAV_TAKEOFF"
                self.get_logger().info(f"start phase -> {self.state}")
            return

        if self.state == "UGV_FORWARD":
            if math.hypot(self.ugv_x - self.ugv_start_x, self.ugv_y - self.ugv_start_y) < self.ugv_distance:
                self._send_ugv(self.ugv_throttle)
            else:
                self._send_ugv(0.0, brake=1.0)
                self.state = "UAV_TAKEOFF"
                self.get_logger().info("UGV moved enough; request UAV takeoff")
            return

        if self.state == "UAV_TAKEOFF":
            self._release_uav()
            if not self.takeoff_requested:
                self.takeoff_requested = True
                self.get_logger().info("UAV takeoff requested")
            if self._request_px4_takeoff_start():
                self.takeoff_done = True
                self.hover_z = self.uav_z
                self.state = "UAV_ASCEND"
                self.get_logger().info("UAV takeoff done; start ascend")
            return

        if self.state == "UAV_ASCEND":
            cmd = VelCmd()
            # 整套项目锚定 MAVROS ENU/FLU: z 轴向上, 上升对应 vz>0.
            cmd.twist.linear.z = self.uav_ascend_speed
            # 旧实现 (NED, 上升对应 vz<0):
            #     cmd.twist.linear.z = -self.uav_ascend_speed
            self.target_cmd_pub.publish(cmd)
            self._publish_state(STATE_GO_TO_TARGET)
            if self.uav_z - self.hover_z >= self.uav_height - 0.1:
                self.state = "FLY_TO_TARGET"
                self.get_logger().info("UAV reached target height; start cruise")
            return

        if self.state == "FLY_TO_TARGET":
            # 障碍判定独立于 DWB 是否在出指令: 只要 scan 看到近物就立即切 AVOID,
            # 由状态机决定该用 DWB 速度还是悬停; 永远不回退到 cruise.
            now = time.time()
            dwb_ready = (now - self.last_dwb_cmd_time) <= self.dwb_cmd_timeout

            # 避障迟滞逻辑 (可复用方法).
            self._update_obstacle_avoidance_state()

            if self.avoid_active:
                # AVOID 时不再发 cruise, 否则状态机切换的窗口里有可能转发 cruise
                # 把飞机推向障碍. avoid 候选指令由 _dwb_cmd_cb 自行发布;
                # DWB 没在工作时, 状态机会用 hover 兜底.
                if not dwb_ready:
                    self.get_logger().warn(
                        f"AVOID active but DWB silent for "
                        f"{now - self.last_dwb_cmd_time:.2f}s "
                        f"(min_scan={self.min_scan_range:.2f}m); state machine will hover.",
                        throttle_duration_sec=2.0,
                    )
                self._publish_state(STATE_AVOID_OBSTACLE)
                # 仍然刷 dwb_goal_pose, 让 DWB 一旦上线就能立刻接管.
                self._publish_dwb_goal_only()
                return

            # 无障碍: 正常发 cruise + goal, 走 GO_TO_TARGET.
            dist = self._publish_target_cmd()
            if dist <= self.target_tolerance:
                self.get_logger().info(
                    f"target reached within {dist:.2f}m; switching to vertical landing"
                )
                self.state = "LANDING"
                self._publish_state(STATE_LANDING)
            else:
                self._publish_state(STATE_GO_TO_TARGET)
            return

        if self.state == "LANDING":
            self._send_ugv(0.0, brake=1.0)
            height = self._publish_landing_cmd()
            self._publish_state(STATE_LANDING)
            if height <= self.landing_z_tolerance:
                self.get_logger().info(
                    f"landed (z={self.uav_z:.2f}m); holding zero command"
                )
                self.state = "DONE"
            return

        if self.state == "DONE":
            self._send_ugv(0.0, brake=1.0)
            self._publish_zero_cmd()
            self._publish_state(STATE_LANDING)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = UgvThenUavNode()
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
