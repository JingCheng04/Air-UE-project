"""MPC 降落协调节点 (PX4 原生版本).

状态机:
  WAIT_CRUISE  -> UAV 升到 cruise_altitude 视为就位.
  HOLD_HOVER   -> 悬停 hover_seconds, 等 yoloe 锁定目标.
  MPC_LANDING  -> 启用 coni-mpc 桥接, 由 MPC 跟踪 AGV 并降到 descend_final;
                  目标持续丢失 target_lost_timeout 秒 -> 释放桥接,
                  保持当前高度回到 HOLD_HOVER 重新等待视觉锁定后再接管.
  DONE         -> 触地完成, 维持桥接 enable 把 UAV 压在原位, 不释放外层.

输出三路话题给 coni-mpc:
  /uav/coni_mpc/quad_odom   nav_msgs/Odometry  UAV ENU/FLU odom (直接转发).
  /uav/coni_mpc/car_odom    nav_msgs/Odometry  AGV ENU 世界系 odom.
  /uav/coni_mpc/imu         sensor_msgs/Imu    AGV 机体系 (FLU) IMU.
  /uav/coni_mpc/bridge_enable std_msgs/Bool    桥接启用信号.

数据来源:
  * UAV odom: /mavros/local_position/odom (PX4 EKF2, ENU/FLU).
  * AGV 位置: 优先 yoloe target_pose, 失流退化为 AGV odom_local (NED->ENU).
  * AGV IMU: agv_imu_odom_node 发布的 FRD/NED 加速度, 转 ENU/FLU.
  * AGV 姿态/速度: 简化为 identity + 0 (CoNi-MPC 仍能收敛到 AGV 上方).
"""

from __future__ import annotations

import math
import threading
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


# ---------------------------------------------------------------------------
# 坐标系换算工具
# ---------------------------------------------------------------------------
# AirSim 世界 NED (x=北, y=东, z=下) -> ROS 世界 ENU (x=东, y=北, z=上).
# AirSim 机体 FRD (x=前, y=右, z=下) -> ROS 机体 FLU (x=前, y=左, z=上).

def ned_to_enu(x: float, y: float, z: float) -> tuple[float, float, float]:
    return (y, x, -z)


def frd_to_flu(x: float, y: float, z: float) -> tuple[float, float, float]:
    return (x, -y, -z)


def quat_rotate(q: Quaternion, v: tuple[float, float, float]) -> tuple[float, float, float]:
    """用四元数 q 把向量 v 从机体系旋转到世界系 (Hamilton 习惯).

    简化的 v' = q * v * q^{-1} 展开公式. q 必须是单位四元数.
    """
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    vx, vy, vz = v
    # 复用经典公式: v' = v + 2 q_v x (q_v x v + q_w v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


# ---------------------------------------------------------------------------
# 协调节点
# ---------------------------------------------------------------------------

class MpcLandCoordinator(Node):
    """编排接管: WAIT_CRUISE -> HOLD_HOVER -> MPC_LANDING -> DONE.

    目标丢失超过 ``target_lost_timeout`` -> 释放桥接, 保持当前高度
    回到 HOLD_HOVER 重新等待视觉锁定后再次接管. 不做爬升, 不做分段下降.
    """

    # 状态机标识 (小写避免和 chase 状态字符串冲突).
    S_WAIT_CRUISE = "wait_cruise"
    S_HOLD_HOVER = "hold_hover"
    S_MPC_LANDING = "mpc_landing"
    S_DONE = "done"

    def __init__(self) -> None:
        super().__init__("mpc_land_coordinator_node")

        # ---- 节点参数 ----
        self.declare_parameter("agv_prefix", "/sim_ugv/airsim_node/UGV_1")
        self.declare_parameter("agv_imu_name", "UGV_1_Imu")
        self.declare_parameter("yoloe_prefix", "/uav/yoloe")
        # 相机相对 UAV 机体的安装俯仰角 (deg).
        self.declare_parameter("camera_pitch_deg", -45.0)
        # 进入 MPC 接管前的悬停时长 (秒).
        self.declare_parameter("hover_seconds", 15.0)
        # 巡航高度阈值 (m): UAV odom z 超过该值视为升空到位 (ENU).
        self.declare_parameter("cruise_altitude", 7.0)
        # MPC 接管期间转发频率 (Hz).
        self.declare_parameter("publish_rate", 30.0)
        # 触地判定: UAV 与 AGV 高度差小于该值 -> 落地完成.
        self.declare_parameter("touch_down_dz", 0.6)
        # yoloe 多久没更新视为失流, 期间退回 AGV odom_local.
        # 1.5s 留够 yoloe 偶发掉帧 + chase 短时偏航的容忍窗口.
        self.declare_parameter("yoloe_stale_sec", 1.5)
        # 目标丢失超过该秒数 -> 回退到 HOLD_HOVER 重新寻找目标.
        self.declare_parameter("target_lost_timeout", 2.0)
        # MPC 跟踪的目标相对高度 (m): coni_mpc fixed_z.
        self.declare_parameter("descend_final", 0.30)
        # 输入 / 输出话题.
        self.declare_parameter("uav_odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("bridge_enable_topic", "/uav/coni_mpc/bridge_enable")
        self.declare_parameter("coni_mpc_prefix", "/uav/coni_mpc")

        agv_prefix = str(self.get_parameter("agv_prefix").value).rstrip("/")
        imu_name = str(self.get_parameter("agv_imu_name").value)
        yoloe_prefix = str(self.get_parameter("yoloe_prefix").value).rstrip("/")
        self.cam_pitch_rad = math.radians(self._float_param("camera_pitch_deg"))
        self.hover_seconds = max(0.0, self._float_param("hover_seconds"))
        self.cruise_altitude = max(1.0, self._float_param("cruise_altitude"))
        self.touch_down_dz = max(0.05, self._float_param("touch_down_dz"))
        self.yoloe_stale_sec = max(0.0, self._float_param("yoloe_stale_sec"))
        self.target_lost_timeout = max(0.1, self._float_param("target_lost_timeout"))
        self.descend_final = max(0.1, self._float_param("descend_final"))
        rate_hz = max(1.0, self._float_param("publish_rate"))
        uav_odom_topic = str(self.get_parameter("uav_odom_topic").value)
        self.bridge_enable_topic = str(self.get_parameter("bridge_enable_topic").value)
        coni_prefix = str(self.get_parameter("coni_mpc_prefix").value).rstrip("/")

        # ---- 状态 ----
        self._lock = threading.Lock()
        self._state = self.S_WAIT_CRUISE
        self._t_hover_started: Optional[float] = None
        self._lost_target_since: Optional[float] = None  # MPC 阶段目标丢失起点

        # 订阅缓存.
        self._uav_odom: Optional[Odometry] = None
        self._agv_odom_ned: Optional[Odometry] = None
        self._agv_imu_frd: Optional[Imu] = None
        self._yoloe_pose: Optional[PoseStamped] = None
        self._yoloe_t: float = -1.0

        # 低空盲飞使用: 最近一次 yoloe 新鲜时记录的 AGV ENU 世界位置 + 时间.
        # 仅在高度 < 2m 且 yoloe 丢失时, 用它做 IMU 位移外推.
        self._last_known_agv_pos: Optional[tuple[float, float, float]] = None
        self._last_known_agv_t: float = -1.0
        # 当前 tick 由 IMU 推算出的 AGV 位置 (ENU). 进入低空盲飞时被设置,
        # 其他时刻为 None; ``_agv_position_enu`` 看到非 None 时优先使用它.
        self._imu_estimated_agv_pos: Optional[tuple[float, float, float]] = None
        # 低空盲飞使用的高度阈值 (m, 传感器 ENU z).
        self.imu_blind_alt = 2.0
        # AGV 速度差分: 用上一帧 AGV ENU 位置 + 时间戳, 估算当前水平速度,
        # 写入 car_odom.twist.linear, 让 CoNi-MPC 能把 AGV 速度作为
        # 已知扰动喂进 MPC, 抑制跟踪移动目标的稳态滞后.
        self._prev_agv_pos: Optional[tuple[float, float, float]] = None
        self._prev_agv_t: float = -1.0

        # ---- ROS2 set_parameters 客户端: 动态调 coni_mpc 的 fixed_z ----
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType
        self._SetParameters = SetParameters
        self._RclParameter = RclParameter
        self._ParameterValue = ParameterValue
        self._ParameterType = ParameterType
        self._param_client = self.create_client(
            SetParameters, "/coni_mpc_controller/set_parameters"
        )

        # ---- 订阅 ----
        sensor_qos = qos_profile_sensor_data
        relay_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, uav_odom_topic,
                                 self._on_uav_odom, sensor_qos)
        self.create_subscription(Odometry, f"{agv_prefix}/odom_local",
                                 self._on_agv_odom, sensor_qos)
        self.create_subscription(Imu, f"{agv_prefix}/imu/{imu_name}",
                                 self._on_agv_imu, sensor_qos)
        self.create_subscription(PoseStamped, f"{yoloe_prefix}/target_pose",
                                 self._on_yoloe_pose, sensor_qos)

        # ---- 发布 ----
        self.quad_odom_pub = self.create_publisher(
            Odometry, f"{coni_prefix}/quad_odom", relay_qos)
        self.car_odom_pub = self.create_publisher(
            Odometry, f"{coni_prefix}/car_odom", relay_qos)
        self.imu_pub = self.create_publisher(
            Imu, f"{coni_prefix}/imu", relay_qos)
        self.enable_pub = self.create_publisher(
            Bool, self.bridge_enable_topic, 10)

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"coordinator ready: hover={self.hover_seconds:.1f}s, "
            f"final={self.descend_final:.2f}m, lost_timeout={self.target_lost_timeout:.1f}s; "
            f"yoloe={yoloe_prefix}/target_pose; out_prefix={coni_prefix}"
        )

    # ------------------------------------------------------------------
    # 订阅回调 (写入加锁, 保证 _tick 线程安全读取)
    # ------------------------------------------------------------------
    def _on_uav_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._uav_odom = msg

    def _on_agv_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._agv_odom_ned = msg

    def _on_agv_imu(self, msg: Imu) -> None:
        with self._lock:
            self._agv_imu_frd = msg

    def _on_yoloe_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._yoloe_pose = msg
            self._yoloe_t = self._now()

    # ------------------------------------------------------------------
    # 数据合成: yoloe + UAV ENU 姿态 -> AGV 在 ENU 世界系下的位置.
    # ------------------------------------------------------------------
    def _agv_position_enu(self, uav_odom: Odometry) -> Optional[tuple[float, float, float]]:
        """返回 AGV 在 ENU 世界系下的位置.

        优先级: yoloe (新鲜) -> 低空 IMU 推算 (由 _tick 注入) -> AGV odom_local.
        在 yoloe 命中时同时记录 ``_last_known_agv_pos`` / ``_last_known_agv_t``,
        供低空盲飞外推使用.
        """
        with self._lock:
            yoloe = self._yoloe_pose
            yoloe_t = self._yoloe_t
            agv_ned = self._agv_odom_ned
        now = self._now()

        if yoloe is not None and (now - yoloe_t) <= self.yoloe_stale_sec:
            sp = math.sin(self.cam_pitch_rad)
            cp = math.cos(self.cam_pitch_rad)
            x_cam = float(yoloe.pose.position.x)
            y_cam = float(yoloe.pose.position.y)
            z_cam = float(yoloe.pose.position.z)
            # 相机系 -> 机体 FRD -> FLU -> ENU.
            body_frd_x = sp * y_cam + cp * z_cam
            body_frd_y = x_cam
            body_frd_z = cp * y_cam - sp * z_cam
            body_flu = frd_to_flu(body_frd_x, body_frd_y, body_frd_z)
            world_enu = quat_rotate(uav_odom.pose.pose.orientation, body_flu)
            agv_pos = (
                float(uav_odom.pose.pose.position.x) + world_enu[0],
                float(uav_odom.pose.pose.position.y) + world_enu[1],
                float(uav_odom.pose.pose.position.z) + world_enu[2],
            )
            # 记录最近一次可信的 AGV 位置, 供低空盲飞外推使用.
            self._last_known_agv_pos = agv_pos
            self._last_known_agv_t = now
            return agv_pos

        # 低空盲飞: yoloe 失流时由 _tick 预先填好 IMU 推算位置, 优先使用.
        if self._imu_estimated_agv_pos is not None:
            return self._imu_estimated_agv_pos

        if agv_ned is not None:
            ex, ey, ez = ned_to_enu(
                float(agv_ned.pose.pose.position.x),
                float(agv_ned.pose.pose.position.y),
                float(agv_ned.pose.pose.position.z),
            )
            return (ex, ey, ez)
        return None

    def _build_car_odom_enu(self, uav_odom: Odometry) -> Optional[Odometry]:
        pos = self._agv_position_enu(uav_odom)
        if pos is None:
            return None
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world_enu"
        msg.child_frame_id = "agv_body"
        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]
        # 简化: AGV 姿态用 identity. 角速度保持 0 (路面 AGV 偏航变化慢,
        # 噪声放大代价高于收益). 平移速度由位置差分给出, 抑制 MPC 滞后.
        msg.pose.pose.orientation.w = 1.0
        vx, vy = self._estimate_agv_velocity(pos)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        # AGV 沿地面运动, 垂直速度近似 0.
        msg.twist.twist.linear.z = 0.0
        return msg

    def _estimate_agv_velocity(self, pos: tuple[float, float, float]) -> tuple[float, float]:
        """对 AGV ENU 位置做有限差分得到水平速度 (vx, vy).

        - 第一次 (无历史) 或 dt 异常 (<=0 / 过大>1s) 时返回 0.
        - 限速 5 m/s, 防止 yoloe 偶发跳变把瞬时速度炸到几十米.
        """
        now = self._now()
        prev_pos = self._prev_agv_pos
        prev_t = self._prev_agv_t
        # 更新缓存供下一帧使用.
        self._prev_agv_pos = pos
        self._prev_agv_t = now
        if prev_pos is None or prev_t < 0:
            return (0.0, 0.0)
        dt = now - prev_t
        if dt <= 0.0 or dt > 1.0:
            return (0.0, 0.0)
        vx = (pos[0] - prev_pos[0]) / dt
        vy = (pos[1] - prev_pos[1]) / dt
        # 限速, 防 yoloe 跳变 / agv_odom 跳点造成数百 m/s 异常.
        speed = math.hypot(vx, vy)
        if speed > 5.0:
            scale = 5.0 / speed
            vx *= scale
            vy *= scale
        return (vx, vy)

    def _build_imu_flu(self) -> Optional[Imu]:
        """AGV IMU (AirSim NED/FRD) -> ROS ENU/FLU."""
        with self._lock:
            src = self._agv_imu_frd
        if src is None:
            return None
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "agv_body_flu"
        # linear_acceleration / angular_velocity 都在机体系, 用 FRD->FLU.
        ax, ay, az = frd_to_flu(
            float(src.linear_acceleration.x),
            float(src.linear_acceleration.y),
            float(src.linear_acceleration.z),
        )
        gx, gy, gz = frd_to_flu(
            float(src.angular_velocity.x),
            float(src.angular_velocity.y),
            float(src.angular_velocity.z),
        )
        out.linear_acceleration = Vector3(x=ax, y=ay, z=az)
        out.angular_velocity = Vector3(x=gx, y=gy, z=gz)
        # orientation 字段 coni-mpc 不读, 给 identity 即可.
        out.orientation.w = 1.0
        # 协方差不可用标记, 与 agv_imu_odom_node 一致.
        out.orientation_covariance[0] = -1.0
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        return out

    # ------------------------------------------------------------------
    # 主循环: 状态机 + 数据转发
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = self._now()
        with self._lock:
            uav_odom = self._uav_odom

        # 1) 等 UAV 升空到巡航高度 (ENU 下 z 为正).
        if self._state == self.S_WAIT_CRUISE:
            if uav_odom is not None and float(uav_odom.pose.pose.position.z) >= self.cruise_altitude:
                self._enter_hold_hover(now)
            return

        # 2) 悬停跟踪: 等到 hover_seconds 满 + hover 期间至少 yoloe 命中过一次后接管.
        if self._state == self.S_HOLD_HOVER:
            assert self._t_hover_started is not None
            wait = self.hover_seconds
            time_ok = (now - self._t_hover_started) >= wait
            # 接管条件放宽: hover 起算之后 yoloe 至少命中过一次 (无需当前帧 fresh).
            # 这样 chase 切到 AVOID_OBSTACLE 临时偏航导致的瞬时失流不会再阻塞接管.
            seen_during_hover = (
                self._yoloe_t > 0 and self._yoloe_t >= self._t_hover_started
            )
            yoloe_fresh = self._yoloe_fresh(now)
            cur_alt = self._alt_above_agv(uav_odom)
            cur_alt_str = f"{cur_alt:.2f}m" if cur_alt is not None else "n/a"
            uav_z = (
                f"{uav_odom.pose.pose.position.z:+.2f}" if uav_odom is not None else "n/a"
            )
            yoloe_age = (now - self._yoloe_t) if self._yoloe_t > 0 else float("inf")
            yoloe_age_str = f"{yoloe_age:.2f}s" if yoloe_age != float("inf") else "never"
            self.get_logger().info(
                f"********（{cur_alt_str}）：wait {wait:.1f} s**************"
                f" | uav_z={uav_z} | yoloe_last={yoloe_age_str}"
                f" | seen_during_hover={seen_during_hover}",
                throttle_duration_sec=1.0,
            )
            if time_ok and seen_during_hover:
                self._enter_mpc_landing(now, uav_odom)
            elif time_ok:
                self.get_logger().warn(
                    f"hover {wait:.1f}s elapsed but yoloe never seen since hover start "
                    f"(last={yoloe_age_str}); waiting for first visual lock",
                    throttle_duration_sec=2.0,
                )
            return

        # 3) MPC 接管: 持续转发三路话题, 维持 enable=True;
        #    yoloe 丢失超过 target_lost_timeout -> 回 HOLD_HOVER 重新寻找目标.
        #    例外: 高度 < imu_blind_alt 时, 摄像头丢失也用 IMU 外推位置作为
        #    AGV 位置继续跟踪降落, 不切状态、不累计丢失计时.
        if self._state == self.S_MPC_LANDING:
            yoloe_fresh = self._yoloe_fresh(now)
            uav_alt_sensor = (
                float(uav_odom.pose.pose.position.z) if uav_odom is not None else None
            )
            low_alt = uav_alt_sensor is not None and uav_alt_sensor < self.imu_blind_alt
            # 默认清空 IMU 推算; 仅在低空盲飞条件下填值.
            self._imu_estimated_agv_pos = None
            if low_alt and not yoloe_fresh and uav_odom is not None:
                est = self._imu_estimate_agv_pos(now, uav_odom)
                if est is not None:
                    self._imu_estimated_agv_pos = est
                    yoloe_fresh = True  # 视作"目标可见", 不切状态.

            self._publish_mpc_inputs(uav_odom)
            self._publish_enable(True)

            if uav_odom is not None and self._reached_touchdown(uav_odom):
                self.get_logger().info("touchdown detected; locking MPC at current altitude")
                self._enter_done()
                return

            if not yoloe_fresh:
                if self._lost_target_since is None:
                    self._lost_target_since = now
                elif (now - self._lost_target_since) >= self.target_lost_timeout:
                    self.get_logger().warn(
                        f"target lost for {self.target_lost_timeout:.1f}s; "
                        "releasing MPC, hovering and searching"
                    )
                    self._enter_hold_hover(now, reset_timer=False)
            else:
                self._lost_target_since = None
            return

        # 4) DONE: 维持桥接 enable, 把 UAV 压在触地高度.
        if self._state == self.S_DONE:
            self._publish_mpc_inputs(uav_odom)
            self._publish_enable(True)
            return

    # ------------------------------------------------------------------
    # 状态切换
    # ------------------------------------------------------------------
    def _enter_hold_hover(self, now: float, reset_timer: bool = True) -> None:
        """进入 / 回到悬停跟踪状态.

        ``reset_timer=True`` 首次到达巡航高度: 重置悬停起算时间.
        ``reset_timer=False`` MPC 阶段目标丢失回退: 释放桥接, 保持当前高度.
        """
        self._state = self.S_HOLD_HOVER
        if reset_timer or self._t_hover_started is None:
            self._t_hover_started = now
        self._lost_target_since = None
        # 释放桥接, 让外层 wrapper 维持当前高度悬停.
        self._publish_enable(False)
        self.get_logger().info(
            f"holding hover ({self.hover_seconds:.1f}s) before (re-)engaging MPC"
        )

    def _enter_mpc_landing(self, now: float, uav_odom: Optional[Odometry]) -> None:
        self._state = self.S_MPC_LANDING
        self._lost_target_since = None
        # 直接把 coni_mpc 的 fixed_z 设为最终目标, 由 MPC 自己生成下降轨迹.
        self._set_fixed_z(self.descend_final)
        self.get_logger().info(
            f"engaging coni_mpc bridge for landing; fixed_z={self.descend_final:.2f}m"
        )

    def _enter_done(self) -> None:
        """触地完成: 维持桥接 enable, 把 fixed_z 压到触地高度,
        让 MPC 持续把无人机压在原位, 不释放给外层避免被拉起.
        """
        self._state = self.S_DONE
        self._publish_enable(True)
        self.get_logger().info(f"MPC landing finished; locking fixed_z={self.descend_final:.2f}m")

    # ------------------------------------------------------------------
    # 发布工具
    # ------------------------------------------------------------------
    def _publish_mpc_inputs(self, uav_odom: Optional[Odometry]) -> None:
        if uav_odom is None:
            return
        car = self._build_car_odom_enu(uav_odom)
        imu = self._build_imu_flu()
        if car is None or imu is None:
            self.get_logger().warn(
                "missing AGV pose or IMU; coni_mpc inputs not published yet",
                throttle_duration_sec=2.0,
            )
            return
        # quad_odom: /mavros/local_position/odom 已经是 ENU/FLU, 直接转发.
        # 但 mavros 写入的 header.stamp 与本节点 ROS now 可能不在同一时钟域
        # (PX4 SITL / system_time / sim_time 不一致), 导致 coni_mpc_controller
        # 用 max_msg_age 判 stamp_is_fresh 时永远过期, 不发 AttitudeTarget.
        # 这里把三路的 header.stamp 全部对齐到协调器 ROS now, 让阈值可控.
        now_stamp = self.get_clock().now().to_msg()
        uav_odom.header.stamp = now_stamp
        car.header.stamp = now_stamp
        imu.header.stamp = now_stamp
        self.quad_odom_pub.publish(uav_odom)
        self.car_odom_pub.publish(car)
        self.imu_pub.publish(imu)

    def _set_fixed_z(self, z: float) -> None:
        """通过 set_parameters 动态调 coni_mpc_controller 的 fixed_z."""
        if not self._param_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                "coni_mpc_controller set_parameters service not available",
                throttle_duration_sec=5.0,
            )
            return
        param = self._RclParameter()
        param.name = "fixed_z"
        param.value = self._ParameterValue()
        param.value.type = self._ParameterType.PARAMETER_DOUBLE
        param.value.double_value = z
        req = self._SetParameters.Request()
        req.parameters = [param]
        self._param_client.call_async(req)

    def _publish_enable(self, enable: bool) -> None:
        msg = Bool()
        msg.data = enable
        self.enable_pub.publish(msg)

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    def _float_param(self, name: str) -> float:
        value: Any = self.get_parameter(name).value
        return float(value)

    def _yoloe_fresh(self, now: float) -> bool:
        with self._lock:
            return (self._yoloe_pose is not None
                    and (now - self._yoloe_t) <= self.yoloe_stale_sec)

    def _imu_estimate_agv_pos(
        self, now: float, uav_odom: Odometry
    ) -> Optional[tuple[float, float, float]]:
        """低空 (高度<2m) 摄像头丢失时, 用 AGV IMU 做最简单的位置外推.

        IMU 注意事项 (来自 agv_imu_odom_node):
          ``linear_acceleration`` 是 *含重力* 的机体系比力. 但 AGV 在地面
          上行驶, 机体近似水平, 重力主要落在 IMU z 轴; 转到 ENU 后水平
          x/y 分量受影响很小, 直接忽略即可. z 维度本就不需要外推 (AGV
          高度不变), 沿用上次 yoloe 命中时的 z 即可.

        步骤:
          1. 取最近一次 yoloe 命中位置 ``_last_known_agv_pos`` 作为基点.
          2. AGV IMU 加速度 NED -> ENU, 仅取水平分量.
          3. 假设上次命中时 AGV 速度近似为 0, 二次积分 dx≈0.5*a*dt^2.
          4. dt 过大 (>5s) 或缺数据 -> 放弃.
        """
        if self._last_known_agv_pos is None or self._last_known_agv_t < 0:
            return None
        dt = now - self._last_known_agv_t
        if dt <= 0.0 or dt > 5.0:
            return None
        with self._lock:
            imu = self._agv_imu_frd
        if imu is None:
            return None
        # NED -> ENU 加速度. 只取水平 (x, y), z 含重力, 忽略.
        ax_enu, ay_enu, _ = ned_to_enu(
            float(imu.linear_acceleration.x),
            float(imu.linear_acceleration.y),
            float(imu.linear_acceleration.z),
        )
        bx, by, bz = self._last_known_agv_pos
        half_dt2 = 0.5 * dt * dt
        return (
            bx + ax_enu * half_dt2,
            by + ay_enu * half_dt2,
            bz,  # AGV 高度保持上次 yoloe 命中值, 避免 IMU 重力分量污染.
        )

    def _alt_above_agv(self, uav_odom: Optional[Odometry]) -> Optional[float]:
        """UAV 相对 AGV 的高度 (m). 缺 AGV 位置时退化为 UAV ENU z."""
        if uav_odom is None:
            return None
        agv_pos = self._agv_position_enu(uav_odom)
        if agv_pos is None:
            return float(uav_odom.pose.pose.position.z)
        return float(uav_odom.pose.pose.position.z) - agv_pos[2]

    def _reached_touchdown(self, uav_odom: Odometry) -> bool:
        alt = self._alt_above_agv(uav_odom)
        return alt is not None and abs(alt) <= self.touch_down_dz

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self) -> None:  # type: ignore[override]
        # 退出时确保桥接失能, 让控制权回到 state_machine velocity setpoint.
        try:
            self._publish_enable(False)
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MpcLandCoordinator()
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
