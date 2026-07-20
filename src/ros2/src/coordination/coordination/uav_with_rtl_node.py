"""带返航与 MPC 跟踪降落的节点
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any, cast

import rclpy
from airsim_interfaces.msg import VelCmd
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as RclParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import NavSatFix

from coordination.ugv_then_uav_node import STATE_AVOID_OBSTACLE, STATE_GO_TO_TARGET, STATE_LANDING, UgvThenUavNode


# AirSim 世界 NED (北/东/下) -> ROS 世界 ENU (东/北/上)。
def ned_to_enu(x: float, y: float, z: float) -> tuple[float, float, float]:
    return (y, x, -z)


# AirSim 机体 FRD (前/右/下) -> ROS 机体 FLU (前/左/上)。
def frd_to_flu(x: float, y: float, z: float) -> tuple[float, float, float]:
    return (x, -y, -z)


# 把角度归一化到 (-π, π]，避免 yaw 误差跨越 ±π 时突变。
def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# 用单位四元数 q 把机体系向量 v 旋转到世界系。
def quat_rotate(q: Quaternion, v: tuple[float, float, float]) -> tuple[float, float, float]:
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class UavWithRtlNode(UgvThenUavNode):
    # 扩展父类状态机，编排启动绑定、返航追踪和 MPC 降落。
    def __init__(self) -> None:
        super().__init__()

        # 启动时序: bind/warmup -> release -> grace -> OFFBOARD/ARM -> ascend。
        # 绑定期间只由 follow 节点控制仿真运动，避免与 PX4 抢控制权。
        self.declare_parameter("bind_on_startup", True)
        self.declare_parameter("bind_resend_rate", 5.0)
        self.declare_parameter("release_delay_seconds", 0.0)
        self.declare_parameter("release_grace_seconds", 0.3)
        # 返航、视觉捕获和 MPC 降落参数；降落相关默认值与参考协调器一致。
        self.declare_parameter("rtl_landing_pause", 2.0)
        self.declare_parameter("home_tolerance", 1.5)
        self.declare_parameter("approach_enable_distance", 10.0)
        self.declare_parameter("approach_exit_distance", 12.0)
        self.declare_parameter("yoloe_prefix", "/uav/yoloe")
        self.declare_parameter("camera_pitch_deg", -45.0)
        self.declare_parameter("bridge_enable_topic", "/uav/coni_mpc/bridge_enable")
        self.declare_parameter("coni_mpc_prefix", "/uav/coni_mpc")
        self.declare_parameter("agv_imu_name", "UGV_1_Imu")
        self.declare_parameter("touch_down_dz", 0.6)
        self.declare_parameter("yoloe_stale_sec", 1.5)
        self.declare_parameter("target_lost_timeout", 2.0)
        self.declare_parameter("descend_final", 0.30)
        # 进入检测范围后，满足悬停时长且期间有视觉命中才接管 MPC。
        self.declare_parameter("hover_seconds", 15.0)
        self.declare_parameter("yaw_align_tolerance", 0.35)
        # MPC 阶段低空视觉短时丢失时，允许用 IMU 外推 AGV 位置。
        self.declare_parameter("imu_blind_alt", 2.0)

        # 读取并约束运行参数。
        self._bind_on_startup = bool(self.get_parameter("bind_on_startup").value)
        bind_rate = self._float_param("bind_resend_rate")
        self._release_delay = max(0.0, self._float_param("release_delay_seconds"))
        self._release_grace = max(0.0, self._float_param("release_grace_seconds"))
        self.rtl_landing_pause = self._float_param("rtl_landing_pause")
        self.home_tolerance = self._float_param("home_tolerance")
        self.approach_enable_distance = max(0.5, self._float_param("approach_enable_distance"))
        self.approach_exit_distance = max(
            self.approach_enable_distance,
            self._float_param("approach_exit_distance"),
        )
        self.cam_pitch_rad = math.radians(self._float_param("camera_pitch_deg"))
        self.touch_down_dz = max(0.05, self._float_param("touch_down_dz"))
        self.yoloe_stale_sec = max(0.0, self._float_param("yoloe_stale_sec"))
        self.target_lost_timeout = max(0.1, self._float_param("target_lost_timeout"))
        self.descend_final = max(0.1, self._float_param("descend_final"))
        self.hover_seconds = max(0.0, self._float_param("hover_seconds"))
        self.yaw_align_tolerance = max(0.05, self._float_param("yaw_align_tolerance"))
        self.imu_blind_alt = max(0.1, self._float_param("imu_blind_alt"))
        yoloe_prefix = self._str_param("yoloe_prefix").rstrip("/")
        coni_prefix = self._str_param("coni_mpc_prefix").rstrip("/")
        self.bridge_enable_topic = self._str_param("bridge_enable_topic")
        agv_imu_name = self._str_param("agv_imu_name")

        # AGV 启动延迟由 launch 控制，父类的 UGV_FORWARD 阶段无需等待。
        self.ugv_distance = 0.0

        # 周期重发 bind，兼容 follow 节点晚于本节点启动的情况。
        self._bind_active = self._bind_on_startup
        if self._bind_on_startup:
            self._send_recovery_cmd("bind")
            period = max(0.05, 1.0 / max(bind_rate, 0.1))
            self.create_timer(period, self._bind_resend_tick)

        # 起飞释放和宽限期分开计时，确保 simSetKinematics 停止后再交给 PX4。
        self._takeoff_entered_at: float | None = None
        self._release_sent = False
        self._release_grace_until: float | None = None
        self._takeoff_ground_z: float | None = None
        self._home_lat: float | None = None
        self._home_lon: float | None = None
        self._landing_pause_start: float | None = None
        self._agv_lat: float | None = None
        self._agv_lon: float | None = None
        self._last_yaw_cmd: float | None = None
        self._rtl_agv_target_lat: float | None = None
        self._rtl_agv_target_lon: float | None = None
        self._last_rtl_goal_lat: float | None = None
        self._last_rtl_goal_lon: float | None = None
        self._rtl_goal_refresh_distance = 1.0

        # 缓存 UAV、AGV、视觉和 MPC 失流状态。
        self._lock = threading.Lock()
        self._uav_odom_msg: Odometry | None = None
        self._agv_odom_ned: Odometry | None = None
        self._agv_imu_frd: Imu | None = None
        self._yoloe_pose: PoseStamped | None = None
        self._yoloe_t = -1.0
        self._lost_target_since: float | None = None
        self._last_known_agv_pos: tuple[float, float, float] | None = None
        self._last_known_agv_t = -1.0
        self._imu_estimated_agv_pos: tuple[float, float, float] | None = None
        self._prev_agv_pos: tuple[float, float, float] | None = None
        self._prev_agv_t = -1.0
        # HOLD_HOVER 起算时间；MPC 失流回退时保留原计时。
        self._t_hover_started: float | None = None
        self._is_on_ground = False
        self._px4_armed = False
        self._rtl_arm_confirm_until = 0.0

        # 飞行状态、AGV 传感器和视觉目标输入。
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._on_uav_odom, qos_profile_sensor_data)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_extended_state, qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self._on_px4_state, 10)
        self.create_subscription(NavSatFix, f"{self.ugv_prefix}/global_gps", self._on_agv_gps, qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{self.ugv_prefix}/odom_local", self._on_agv_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, f"{self.ugv_prefix}/imu/{agv_imu_name}", self._on_agv_imu, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f"{yoloe_prefix}/target_pose", self._on_yoloe_pose, qos_profile_sensor_data)

        # 返航进入检测/跟踪阶段后发布 AGV yaw setpoint。
        self.yaw_setpoint_pub = self.create_publisher(Float32, "/uav/control/yaw_setpoint", 10)

        # 向 CoNi-MPC 转发统一坐标系下的状态，并控制姿态桥接。
        relay_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.quad_odom_pub = self.create_publisher(Odometry, f"{coni_prefix}/quad_odom", relay_qos)
        self.car_odom_pub = self.create_publisher(Odometry, f"{coni_prefix}/car_odom", relay_qos)
        self.imu_pub = self.create_publisher(Imu, f"{coni_prefix}/imu", relay_qos)
        self.enable_pub = self.create_publisher(Bool, self.bridge_enable_topic, 10)
        self._param_client = self.create_client(SetParameters, "/coni_mpc_controller/set_parameters")

    def _send_recovery_cmd(self, value: str) -> None:
        # 通过 follow 节点的 recovery_cmd 发送 bind/release。
        msg = String()
        msg.data = value
        self.recovery_cmd_pub.publish(msg)

    def _bind_resend_tick(self) -> None:
        if not self._bind_active:
            return
        if self.state in ("WAIT_READY", "WAIT_START_DELAY"):
            self._send_recovery_cmd("bind")
        else:
            self._bind_active = False

    def _on_uav_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._uav_odom_msg = msg

    def _on_agv_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._agv_odom_ned = msg

    def _on_agv_gps(self, msg: NavSatFix) -> None:
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        self._agv_lat = float(msg.latitude)
        self._agv_lon = float(msg.longitude)

    def _on_extended_state(self, msg: ExtendedState) -> None:
        # 用 landed_state 辅助判定接地，避免仅依赖 odom 高度。
        self._is_on_ground = (msg.landed_state == 1)

    def _on_px4_state(self, msg: State) -> None:
        self._px4_armed = bool(msg.armed)

    def _have_agv_gps(self) -> bool:
        return self._agv_lat is not None and self._agv_lon is not None

    def _xy_distance_to_agv(self) -> float:
        assert self._agv_lat is not None and self._agv_lon is not None
        lat = math.radians(self.uav_lat)
        north = math.radians(self._agv_lat - self.uav_lat) * 6378137.0
        east = math.radians(self._agv_lon - self.uav_lon) * 6378137.0 * math.cos(lat)
        return math.hypot(east, north)

    def _refresh_rtl_goal_if_changed(self) -> None:
        """AGV 移动超过阈值时刷新 DWB 目标。"""
        if not self._have_agv_gps():
            return
        assert self._agv_lat is not None and self._agv_lon is not None
        lat = float(self._agv_lat)
        lon = float(self._agv_lon)
        if self._last_rtl_goal_lat is None or self._last_rtl_goal_lon is None:
            changed = True
            moved = 0.0
        else:
            lat_scale = 6378137.0
            north = math.radians(lat - self._last_rtl_goal_lat) * lat_scale
            east = math.radians(lon - self._last_rtl_goal_lon) * lat_scale * math.cos(math.radians(lat))
            moved = math.hypot(east, north)
            changed = moved >= self._rtl_goal_refresh_distance
        if not changed:
            return
        self._last_rtl_goal_lat = lat
        self._last_rtl_goal_lon = lon
        self._publish_dwb_goal_only()
        self.get_logger().info(
            f"RTL refreshed moving-AGV DWB goal (vehicle_delta={moved:.2f}m)",
            throttle_duration_sec=1.0,
        )

    def _publish_yaw_to_agv(self) -> float | None:
        if not self._have_agv_gps():
            return None
        east, north = self._target_delta_m()
        if math.hypot(east, north) < 0.5 and self._last_yaw_cmd is not None:
            yaw = self._last_yaw_cmd
        else:
            # 按 PX4/AirSim 安装方向修正，使机头摄像头朝向 AGV。
            yaw = wrap_pi(math.atan2(north, east) - math.pi / 2.0)
        self._last_yaw_cmd = yaw
        msg = Float32()
        msg.data = float(yaw)
        self.yaw_setpoint_pub.publish(msg)
        return yaw

    def _yaw_error_to_agv(self) -> float | None:
        if self._last_yaw_cmd is None:
            return None
        return wrap_pi(self._last_yaw_cmd - self.uav_yaw)

    def _yaw_aligned_to_agv(self) -> bool:
        yaw_err = self._yaw_error_to_agv()
        return yaw_err is not None and abs(yaw_err) <= self.yaw_align_tolerance

    def _publish_hover_cmd(self) -> None:
        # 目标数据未就绪时保持高度，避免向错误方向飞行。
        cmd = VelCmd()
        cmd.twist.linear.z = self._height_vz()
        self.target_cmd_pub.publish(cmd)
        self._publish_state(STATE_GO_TO_TARGET)

    def _publish_avoid_or_fallback(self, now: float, dwb_ready: bool) -> None:
        """发布父类兼容的避障状态；DWB 沉默时补发高度保持指令。"""
        if not dwb_ready:
            self.get_logger().warn(
                f"AVOID active but DWB silent for "
                f"{now - self.last_dwb_cmd_time:.2f}s "
                f"(min_scan={self.min_scan_range:.2f}m); "
                f"publishing height-hold fallback",
                throttle_duration_sec=2.0,
            )
            # DWB 无输出时补发水平归零、垂直高度保持的指令。
            hold = VelCmd()
            hold.twist.linear.z = self._height_vz()
            self.avoid_cmd_pub.publish(hold)
        self._publish_state(STATE_AVOID_OBSTACLE)
        self._publish_dwb_goal_only()

    def _on_agv_imu(self, msg: Imu) -> None:
        with self._lock:
            self._agv_imu_frd = msg

    def _on_yoloe_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._yoloe_pose = msg
            self._yoloe_t = self._now()

    def _tick(self) -> None:  # noqa: C901
        # 去程准备阶段仍由父类驱动。
        if self.state == "WAIT_READY":
            super()._tick()
            # 记录返航参考点，仅保存一次。
            if self._home_lat is None and self.have_gps:
                self._home_lat = self.uav_lat
                self._home_lon = self.uav_lon
            return

        if self.state == "WAIT_START_DELAY":
            super()._tick()
            return

        # 首次起飞先释放启动绑定，再交回父类完成飞控握手。
        if self.state == "UAV_TAKEOFF" and not self.takeoff_done:
            # 未启用绑定时完全使用父类起飞流程。
            if not self._bind_on_startup:
                super()._tick()
                return

            now = time.time()

            # 首个 UAV_TAKEOFF tick 释放绑定；启动等待已由 launch 提供。
            if self._takeoff_entered_at is None:
                self._takeoff_entered_at = now
                self._release_uav()
                self._bind_active = False
                self._release_sent = True
                self._release_grace_until = now + self._release_grace
                self.get_logger().info(
                    f"UAV_TAKEOFF entered; releasing startup bind "
                    f"(grace={self._release_grace:.2f}s)"
                )
                return

            # 宽限期内不调用父类，让 follow 节点完成释放。
            if self._release_grace_until is not None and now < self._release_grace_until:
                return

            # 宽限期结束后交给父类完成 OFFBOARD/ARM，并进入单段爬升。
            super()._tick()
            return

        if self.state == "UAV_ASCEND":
            # 爬升阶段只允许上升/保持，超出目标高度时直接进入巡航。
            height_err = self.uav_height - (self.uav_z - self.hover_z)
            if height_err <= 0.2:
                self.state = "FLY_TO_TARGET"
                self.get_logger().info("UAV reached target height; start cruise")
                return
            cmd = VelCmd()
            cmd.twist.linear.z = max(self._height_vz(), 0.0)
            self.target_cmd_pub.publish(cmd)
            self._publish_state(STATE_GO_TO_TARGET)
            return

        if self.state == "FLY_TO_TARGET":
            # 复用父类巡航和避障逻辑，但先更新避障状态以支持 fallback。
            now = time.time()
            dwb_ready = (now - self.last_dwb_cmd_time) <= self.dwb_cmd_timeout
            # 到达任务点优先于避障迟滞，避免目标点继续转发旧 DWB 速度。
            if self.have_gps:
                _, _, dist = self._publish_dwb_goal_only()
                if dist <= self.target_tolerance:
                    self._send_ugv(0.0, brake=1.0)
                    self.state = "LANDING"
                    self._publish_state(STATE_LANDING)
                    self.get_logger().info(
                        f"target reached within {dist:.2f}m; switching to vertical landing"
                    )
                    return

            self._update_obstacle_avoidance_state()
            if self.avoid_active:
                self._publish_avoid_or_fallback(now, dwb_ready)
                return

            dist = self._publish_target_cmd()
            self._publish_state(STATE_GO_TO_TARGET)
            return

        if self.state == "LANDING":
            # 去程降落沿用父类逻辑；这里只处理落地后的返航切换。
            self._send_ugv(0.0, brake=1.0)
            height = self._publish_landing_cmd()
            self._publish_state(STATE_LANDING)
            if height <= self.landing_z_tolerance or self._is_on_ground:
                self.state = "DONE_AT_TARGET"
                self._landing_pause_start = time.time()
                self.get_logger().info("landed at target; pause then RTL")
            return

        if self.state == "DONE_AT_TARGET":
            # 落地后短暂停留；二次起飞时再锁定地面高度。
            self._publish_zero_cmd()
            self._publish_state(STATE_LANDING)
            if self._landing_pause_start is not None and time.time() - self._landing_pause_start >= self.rtl_landing_pause:
                # 重置父类起飞握手；返航阶段不再重复 release。
                self.offboard_requested = False
                self.offboard_future = None
                self.arm_requested = False
                self.arm_future = None
                self.offboard_retry_deadline = 0.0
                self.arm_retry_deadline = 0.0
                self.recovery_released = True
                self._takeoff_ground_z = self.uav_z
                self.state = "RTL_TAKEOFF"
                self.get_logger().info("RTL phase begin -> RTL_TAKEOFF")
            return

        # 返航二次起飞并爬升到巡航高度。
        if self.state == "RTL_TAKEOFF":
            # ARM 完成前保持零速 LANDING 指令，避免未接管时发送爬升速度。
            self._publish_zero_cmd()
            self._publish_state(STATE_LANDING)
            if self._rtl_arm_confirm_until > 0.0:
                now = time.time()
                if self._px4_armed:
                    self._rtl_arm_confirm_until = 0.0
                elif now < self._rtl_arm_confirm_until:
                    return
                else:
                    self._rtl_arm_confirm_until = 0.0
                    self.arm_requested = False
                    self.arm_future = None
                    self.arm_retry_deadline = now + 1.0
                    self.get_logger().warn(
                        "PX4 arm service succeeded but /mavros/state is still armed=false; retrying"
                    )
                    return

            if self._request_px4_takeoff_start() and self._px4_armed:
                # 以 ARM 成功时的地面高度作为返航爬升基准。
                self.hover_z = self._takeoff_ground_z if self._takeoff_ground_z is not None else self.uav_z
                self._takeoff_ground_z = None
                self.state = "RTL_ASCEND"
                self.get_logger().info("RTL takeoff done; start ascend")
            elif self.arm_requested and self.arm_future is not None and self.arm_future.done():
                # 必须等 /mavros/state 确认 armed=true 后才进入爬升。
                self._rtl_arm_confirm_until = time.time() + 0.5
            return

        if self.state == "RTL_ASCEND":
            height_err = self.uav_height - (self.uav_z - self.hover_z)
            if height_err <= 0.2:
                # 爬升完成后，将返航目标切换为当前 AGV GPS。
                if self._have_agv_gps():
                    self._rtl_agv_target_lat = self._agv_lat
                    self._rtl_agv_target_lon = self._agv_lon
                    self.target_lat = self._rtl_agv_target_lat
                    self.target_lon = self._rtl_agv_target_lon
                    self.state = "RTL_FLY_AGV"
                    self.get_logger().info(
                        f"RTL target switched to AGV GPS=({self.target_lat:.8f}, {self.target_lon:.8f})"
                    )
                else:
                    self.get_logger().warn("AGV GPS not ready; hovering until target is available")
                return
            cmd = VelCmd()
            # 返航爬升阶段只允许上升/保持。
            cmd.twist.linear.z = max(self._height_vz(), 0.0)
            self.target_cmd_pub.publish(cmd)
            self._publish_state(STATE_GO_TO_TARGET)
            return

        # GPS 追踪阶段保留避障；检测范围内转入视觉跟踪。
        if self.state == "RTL_FLY_AGV":
            # 远距返航复用去程避障逻辑；进入检测范围前不调整 yaw。
            now = time.time()
            if not self._have_agv_gps():
                self._publish_hover_cmd()
                return

            self.target_lat = self._agv_lat
            self.target_lon = self._agv_lon
            # 进入检测范围后由跟踪阶段持续发布 yaw 对准命令。
            self._refresh_rtl_goal_if_changed()
            dist_to_agv = self._xy_distance_to_agv()
            if dist_to_agv <= self.approach_enable_distance:
                self.state = "RTL_TRACK_AGV"
                if self._t_hover_started is None:
                    self._t_hover_started = self._now()
                self._lost_target_since = None
                self.get_logger().info(
                    f"AGV acquired within {self.approach_enable_distance:.1f}m; "
                    f"holding hover ({self.hover_seconds:.1f}s) before engaging MPC"
                )
                return

            dwb_ready = (now - self.last_dwb_cmd_time) <= self.dwb_cmd_timeout
            self._update_obstacle_avoidance_state()
            if self.avoid_active:
                self._publish_avoid_or_fallback(now, dwb_ready)
                return

            self._publish_target_cmd()
            self._publish_state(STATE_GO_TO_TARGET)
            return

        if self.state == "RTL_TRACK_AGV":
            self._track_agv_step()
            return

        # MPC 接管后处理视觉失流、触地和最终位置保持。
        if self.state == "MPC_HOVER":
            self._mpc_hover_step()
            return

        if self.state == "MPC_LANDING":
            self._mpc_landing_step()
            return

        if self.state == "DONE":
            # 保持 MPC 桥接启用，由 MPC 维持触地位置。
            with self._lock:
                uav_odom = self._uav_odom_msg
            self._publish_mpc_inputs(uav_odom)
            self._publish_enable(True)
            return

        super()._tick()

    def _resume_rtl_tracking(self, reason: str) -> None:
        """回到返程入口，重新执行捕获、悬停和 MPC 降落。"""
        self._publish_enable(False)
        self._lost_target_since = None
        self._t_hover_started = None
        self._imu_estimated_agv_pos = None
        self.state = "RTL_FLY_AGV"
        self.get_logger().warn(reason, throttle_duration_sec=2.0)

    def _mpc_hover_step(self) -> None:
        # 兼容旧状态，复用跟踪阶段的视觉等待逻辑。
        self._track_agv_step()

    def _track_agv_step(self) -> None:
        # 检测范围内持续对准 AGV，并按悬停时长和视觉命中条件接管 MPC。
        self._publish_enable(False)
        if not self._have_agv_gps():
            self._publish_hover_cmd()
            return

        self.target_lat = self._agv_lat
        self.target_lon = self._agv_lon
        self._publish_yaw_to_agv()
        self._refresh_rtl_goal_if_changed()
        dist = self._xy_distance_to_agv()
        if dist > self.approach_exit_distance:
            self._t_hover_started = None
            self.state = "RTL_FLY_AGV"
            self.get_logger().info(
                f"AGV left {self.approach_exit_distance:.1f}m detection range "
                f"(dist={dist:.2f}m); resetting detection and resuming tracking"
            )
            return

        if self._t_hover_started is None:
            self._t_hover_started = self._now()

        # 需要完成悬停时长，且进入本阶段后至少获得一次 YOLOE 命中。
        now_secs = self._now()
        hover_started = self._t_hover_started
        time_ok = hover_started is not None and (now_secs - hover_started) >= self.hover_seconds
        seen_during_hover = (
            hover_started is not None and self._yoloe_t > 0 and self._yoloe_t >= hover_started
        )
        yoloe_age = (now_secs - self._yoloe_t) if self._yoloe_t > 0 else float("inf")
        yoloe_age_str = f"{yoloe_age:.2f}s" if yoloe_age != float("inf") else "never"
        self.get_logger().info(
            f"RTL HOLD_HOVER: wait {self.hover_seconds:.1f}s"
            f" | yoloe_last={yoloe_age_str}"
            f" | seen_during_hover={seen_during_hover}",
            throttle_duration_sec=1.0,
        )

        if time_ok and seen_during_hover:
            self._set_fixed_z(self.descend_final)
            self._lost_target_since = None
            self.state = "MPC_LANDING"
            self.get_logger().info(
                f"HOLD_HOVER done (elapsed={now_secs - hover_started:.1f}s, "
                f"visual={seen_during_hover}); engaging MPC landing"
            )
            return
        if time_ok:
            self.get_logger().warn(
                f"hover {self.hover_seconds:.1f}s elapsed but yoloe never seen since hover start "
                f"(last={yoloe_age_str}); waiting for first visual lock",
                throttle_duration_sec=2.0,
            )

        # 等待视觉锁定期间保持当前位置和高度。
        self._publish_hover_cmd()

    def _mpc_landing_step(self) -> None:
        now = self._now()
        with self._lock:
            uav_odom = self._uav_odom_msg
        yoloe_fresh = self._yoloe_fresh(now)
        # 低于 imu_blind_alt 且视觉失流时，允许 IMU 短时外推。
        uav_alt_sensor = float(uav_odom.pose.pose.position.z) if uav_odom is not None else None
        low_alt = uav_alt_sensor is not None and uav_alt_sensor < self.imu_blind_alt
        self._imu_estimated_agv_pos = None
        if low_alt and not yoloe_fresh and uav_odom is not None:
            est = self._imu_estimate_agv_pos(now)
            if est is not None:
                self._imu_estimated_agv_pos = est
                yoloe_fresh = True  # 视作视觉可用，继续 MPC 降落。

        self._publish_mpc_inputs(uav_odom)
        self._publish_enable(True)

        if uav_odom is not None and self._reached_touchdown(uav_odom):
            self.state = "DONE"
            self.get_logger().info("MPC landing finished; locking at touchdown")
            return

        if not yoloe_fresh:
            if self._lost_target_since is None:
                self._lost_target_since = now
            elif now - self._lost_target_since >= self.target_lost_timeout:
                # 释放 MPC，回到悬停搜索；不重置原悬停计时。
                self._publish_enable(False)
                self._lost_target_since = None
                self.state = "RTL_TRACK_AGV"
                self.get_logger().warn(
                    f"target lost for {self.target_lost_timeout:.1f}s; "
                    "releasing MPC, hovering and searching"
                )
        else:
            self._lost_target_since = None

    def _agv_position_enu(self, uav_odom: Odometry) -> tuple[float, float, float] | None:
        # 按视觉、IMU 外推、AGV odom 的优先级返回 ENU 目标位置。
        with self._lock:
            yoloe = self._yoloe_pose
            yoloe_t = self._yoloe_t
            agv_ned = self._agv_odom_ned
        now = self._now()
        # 优先用 YOLOE 相对位姿，结合相机安装角和 UAV 姿态换算到 ENU。
        if yoloe is not None and now - yoloe_t <= self.yoloe_stale_sec:
            sp = math.sin(self.cam_pitch_rad)
            cp = math.cos(self.cam_pitch_rad)
            x_cam = float(yoloe.pose.position.x)
            y_cam = float(yoloe.pose.position.y)
            z_cam = float(yoloe.pose.position.z)
            body_frd = (sp * y_cam + cp * z_cam, x_cam, cp * y_cam - sp * z_cam)
            world_enu = quat_rotate(uav_odom.pose.pose.orientation, frd_to_flu(*body_frd))
            agv_pos = (
                float(uav_odom.pose.pose.position.x) + world_enu[0],
                float(uav_odom.pose.pose.position.y) + world_enu[1],
                float(uav_odom.pose.pose.position.z) + world_enu[2],
            )
            self._last_known_agv_pos = agv_pos
            self._last_known_agv_t = now
            return agv_pos
        if self._imu_estimated_agv_pos is not None:
            return self._imu_estimated_agv_pos
        if agv_ned is None:
            return None
        # 视觉和 IMU 不可用时退化为 AGV odom_local。
        return ned_to_enu(float(agv_ned.pose.pose.position.x), float(agv_ned.pose.pose.position.y), float(agv_ned.pose.pose.position.z))

    def _publish_mpc_inputs(self, uav_odom: Odometry | None) -> None:
        # 组装并发布 CoNi-MPC 所需的 UAV、AGV 和 IMU 输入。
        if uav_odom is None:
            return
        car = self._build_car_odom_enu(uav_odom)
        imu = self._build_imu_flu()
        if car is None or imu is None:
            return
        # 三路输入使用同一时间戳，避免控制器误判消息过期。
        now_stamp = self.get_clock().now().to_msg()
        uav_odom.header.stamp = now_stamp
        car.header.stamp = now_stamp
        imu.header.stamp = now_stamp
        self.quad_odom_pub.publish(uav_odom)
        self.car_odom_pub.publish(car)
        self.imu_pub.publish(imu)

    def _build_car_odom_enu(self, uav_odom: Odometry) -> Odometry | None:
        # 构造包含目标位置和估计速度的 AGV ENU 里程计。
        pos = self._agv_position_enu(uav_odom)
        if pos is None:
            return None
        msg = Odometry()
        msg.header.frame_id = "world_enu"
        msg.child_frame_id = "agv_body"
        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]
        msg.pose.pose.orientation.w = 1.0
        vx, vy = self._estimate_agv_velocity(pos)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = 0.0
        return msg

    def _estimate_agv_velocity(self, pos: tuple[float, float, float]) -> tuple[float, float]:
        # 用位置差分估计水平速度，作为 MPC 的目标速度输入。
        now = self._now()
        prev_pos, prev_t = self._prev_agv_pos, self._prev_agv_t
        self._prev_agv_pos = pos
        self._prev_agv_t = now
        if prev_pos is None or prev_t < 0.0:
            return (0.0, 0.0)
        dt = now - prev_t
        if dt <= 0.0 or dt > 1.0:
            return (0.0, 0.0)
        vx = (pos[0] - prev_pos[0]) / dt
        vy = (pos[1] - prev_pos[1]) / dt
        return (vx, vy)

    def _build_imu_flu(self) -> Imu | None:
        # 将 AGV IMU 从 FRD 转换为 FLU。
        with self._lock:
            src = self._agv_imu_frd
        if src is None:
            return None
        out = Imu()
        out.header.frame_id = "agv_body_flu"
        ax, ay, az = frd_to_flu(float(src.linear_acceleration.x), float(src.linear_acceleration.y), float(src.linear_acceleration.z))
        gx, gy, gz = frd_to_flu(float(src.angular_velocity.x), float(src.angular_velocity.y), float(src.angular_velocity.z))
        out.linear_acceleration = Vector3(x=ax, y=ay, z=az)
        out.angular_velocity = Vector3(x=gx, y=gy, z=gz)
        out.orientation.w = 1.0
        out.orientation_covariance[0] = -1.0
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        return out

    def _imu_estimate_agv_pos(self, now: float) -> tuple[float, float, float] | None:
        # 低空视觉失流时，从最近一次可信位置做短时二次积分外推。
        if self._last_known_agv_pos is None or self._last_known_agv_t < 0.0:
            return None
        dt = now - self._last_known_agv_t
        if dt <= 0.0 or dt > 5.0:
            return None
        with self._lock:
            imu = self._agv_imu_frd
        if imu is None:
            return None
        ax, ay, _ = ned_to_enu(float(imu.linear_acceleration.x), float(imu.linear_acceleration.y), float(imu.linear_acceleration.z))
        bx, by, bz = self._last_known_agv_pos
        half_dt2 = 0.5 * dt * dt
        return (bx + ax * half_dt2, by + ay * half_dt2, bz)

    def _reached_touchdown(self, uav_odom: Odometry) -> bool:
        # 按 UAV 与 AGV 的垂直距离判断触地。
        agv_pos = self._agv_position_enu(uav_odom)
        if agv_pos is None:
            return False
        return abs(float(uav_odom.pose.pose.position.z) - agv_pos[2]) <= self.touch_down_dz

    def _set_fixed_z(self, z: float) -> None:
        # 动态设置 CoNi-MPC 的目标相对高度。
        if not self._param_client.wait_for_service(timeout_sec=0.1):
            return
        param = RclParameter()
        param.name = "fixed_z"
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=z)
        req = SetParameters.Request()
        req.parameters = [param]
        self._param_client.call_async(req)

    def _publish_enable(self, enable: bool) -> None:
        msg = Bool()
        msg.data = enable
        self.enable_pub.publish(msg)

    def _yoloe_fresh(self, now: float) -> bool:
        with self._lock:
            return self._yoloe_pose is not None and now - self._yoloe_t <= self.yoloe_stale_sec

    def _float_param(self, name: str) -> float:
        value: Any = self.get_parameter(name).value
        return float(value)

    def _str_param(self, name: str) -> str:
        value = self.get_parameter(name).value
        return cast(str, value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self) -> None:  # type: ignore[override]
        try:
            self._publish_enable(False)
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = UavWithRtlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
