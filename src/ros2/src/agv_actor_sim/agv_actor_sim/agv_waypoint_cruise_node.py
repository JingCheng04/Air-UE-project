"""定速巡航节点: 按 YAML 经纬度路径依次访问 AGV 轨迹点.

读取 ``AGV_Traj.yaml`` 中按 ``Cross0..CrossN`` 顺序排列的经纬度坐标, 收到车辆
当前 GPS 后把它们换算到当前 ``odom_local`` 局部坐标系, 然后用一个简单的比例式
航向控制器驱动 ``agv_actor_node``:

    线速度恒定 v_target  (默认 2.5 m/s)
    机体角速度限幅 |w| <= v_target / R_min  (默认 R_min = 3 m)

由 ``|w| <= v / R_min`` 决定的曲率半径满足 R = v / |w| >= R_min, 因此整条
轨迹的瞬时转弯半径都不会小于 3 m.

依赖:
    - ``agv_actor_node`` 已就绪, 把 ``car_cmd`` 转成 actor 运动
    - ``agv_imu_odom_node`` 已就绪, 在 ``<prefix>/odom_local`` 发布位姿反馈

参数:
    waypoints_file        YAML 路径; 留空则使用包内 ``AGV_Traj.yaml``
    topic_prefix          ROS 话题命名空间, 与 actor / imu_odom 一致
    target_speed          巡航线速度 m/s
    min_turn_radius       最小转弯半径 m
    waypoint_tolerance    认为已到达航点的距离阈值 m
    max_speed             actor 节点的 throttle=1 对应线速度
    max_yaw_rate          actor 节点的 steering=1 对应角速度 (deg/s)
    rate                  控制循环频率 Hz
    heading_gain          航向 P 控制增益

巡航顺序固定在 ``CRUISE_ORDER`` 中定义, 循环往复.
"""

from __future__ import annotations

import math
import os
import threading
from typing import Any, List, Tuple, cast

import rclpy
import yaml
from airsim_interfaces.msg import CarControls  # pyright: ignore[reportMissingImports]
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS_M = 6378137.0
ALIGN_HEADING_TOL = math.radians(5.0)

# 固定巡航顺序 (按 Cross 序号), 循环往复; 取消 Cross2 的注释即可恢复三点循环.
CRUISE_ORDER = [
    1,  # Cross1
    # 2,  # Cross2
    0,  # 回到 Cross0
]


def param_value(node: Node, name: str) -> Any:
    return cast(Any, node.get_parameter(name).value)


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """从四元数提取偏航角 (NED, +x=North, +y=East, yaw 绕 +z 向下)."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _waypoint_sort_key(name: str) -> Tuple[int, str]:
    """优先按名字里的整数后缀排序, 没有数字时退化成字典序."""
    digits = "".join(ch for ch in name if ch.isdigit())
    return (int(digits) if digits else 0, name)


class AgvWaypointCruiseNode(Node):
    """按 YAML 路径定速巡航的控制器."""

    def __init__(self) -> None:
        super().__init__("agv_waypoint_cruise_node")

        # 节点参数
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("topic_prefix", "/sim_ugv/airsim_node/UGV_1")
        self.declare_parameter("target_speed", 2.5)         # m/s
        self.declare_parameter("min_turn_radius", 3.0)      # m
        self.declare_parameter("waypoint_tolerance", 1.0)   # m
        # actor 节点的归一化映射, 与 agv_actor_node 的同名参数保持一致
        self.declare_parameter("max_speed", 2.5)            # m/s @ throttle=1
        self.declare_parameter("max_yaw_rate", 90.0)        # deg/s @ steering=1
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("heading_gain", 1.5)

        wp_file = str(param_value(self, "waypoints_file"))
        if not wp_file:
            here = os.path.dirname(os.path.abspath(__file__))
            wp_file = os.path.normpath(os.path.join(here, "..", "AGV_Traj.yaml"))
        self.waypoints_file = wp_file

        prefix = str(param_value(self, "topic_prefix")).rstrip("/")
        self.target_speed = max(0.0, float(param_value(self, "target_speed")))
        self.min_turn_radius = max(0.1, float(param_value(self, "min_turn_radius")))
        self.tol = max(0.1, float(param_value(self, "waypoint_tolerance")))
        self.max_speed = max(0.1, float(param_value(self, "max_speed")))
        self.max_yaw_rate = math.radians(max(1.0, float(param_value(self, "max_yaw_rate"))))
        rate_hz = max(1.0, float(param_value(self, "rate")))
        self.period = 1.0 / rate_hz
        self.heading_gain = max(0.1, float(param_value(self, "heading_gain")))

        if self.target_speed > self.max_speed:
            self.get_logger().warn(
                f"target_speed={self.target_speed:.2f} m/s 超出 max_speed={self.max_speed:.2f} m/s, "
                f"将被 throttle 限幅截到 {self.max_speed:.2f} m/s"
            )

        # 读取经纬度航点; 收到当前 GPS 后再换算到 odom_local.
        self.raw_waypoints: List[Tuple[float, float, str]] = []  # (lat, lon, name)
        self.waypoints: List[Tuple[float, float, str]] = []      # (north, east, name)
        self._load_waypoints(self.waypoints_file)
        if not self.raw_waypoints:
            self.get_logger().error(f"no waypoints loaded from {self.waypoints_file}")
            raise SystemExit(2)

        # 校验 CRUISE_ORDER 里的下标都在已加载航点范围内.
        max_index = len(self.raw_waypoints) - 1
        for i in CRUISE_ORDER:
            if i > max_index:
                self.get_logger().error(
                    f"CRUISE_ORDER references Cross{i} but only 0..{max_index} loaded"
                )
                raise SystemExit(2)
        self.cruise_order = list(CRUISE_ORDER)

        # 状态
        self.idx = 0
        self.have_odom = False
        self.have_gps = False
        self.waypoints_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.lock = threading.Lock()
        # 用来周期性 log + watchdog
        self._tick_count = 0
        self._cmd_count = 0
        self._last_dist = float("inf")

        # ROS 接口
        cmd_topic = f"{prefix}/car_cmd"
        odom_topic = f"{prefix}/odom_local"
        gps_topic = f"{prefix}/global_gps"
        # agv_imu_odom_node 发布 odom_local 用的是 BEST_EFFORT, 这里必须匹配,
        # 否则 ROS2 QoS 不兼容, 订阅端永远拿不到消息.
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_pub = self.create_publisher(CarControls, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, odom_qos)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, odom_qos)

        order_names = ",".join(self.raw_waypoints[i][2] for i in self.cruise_order)
        self.get_logger().info(
            f"loaded {len(self.raw_waypoints)} waypoints from {self.waypoints_file}; "
            f"cruise_order={order_names}, "
            f"v={self.target_speed:.2f} m/s, R_min={self.min_turn_radius:.2f} m, "
            f"tol={self.tol:.2f} m, "
            f"actor max_speed={self.max_speed:.2f} m/s, "
            f"actor max_yaw_rate={math.degrees(self.max_yaw_rate):.1f} deg/s; "
            f"prefix={prefix}; cmd_topic={cmd_topic}; odom_topic={odom_topic}; gps_topic={gps_topic}"
        )
        for lat, lon, name in self.raw_waypoints:
            self.get_logger().info(f"  {name}: latitude={lat:.9f}, longitude={lon:.9f}")

        self.create_timer(self.period, self._tick)

    # ----------------------------------------------------------- waypoints io

    def _load_waypoints(self, path: str) -> None:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"waypoints file {path} must contain a top-level mapping")

        keys = sorted(data.keys(), key=_waypoint_sort_key)
        for k in keys:
            entry = data[k]
            if not isinstance(entry, dict):
                continue
            if "latitude" not in entry or "longitude" not in entry:
                continue
            lat = float(entry["latitude"])
            lon = float(entry["longitude"])
            self.raw_waypoints.append((lat, lon, str(k)))

    # ----------------------------------------------------------- callbacks

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.x = float(p.x)
            self.y = float(p.y)
            self.yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
            self.have_odom = True

    def _on_gps(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        with self.lock:
            self.gps_lat = lat
            self.gps_lon = lon
            self.have_gps = True

    # ----------------------------------------------------------- helpers

    def _send(self, throttle: float, steering: float, brake: float = 0.0) -> None:
        msg = CarControls()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle = float(max(-1.0, min(1.0, throttle)))
        msg.steering = float(max(-1.0, min(1.0, steering)))
        msg.brake = float(max(0.0, min(1.0, brake)))
        msg.handbrake = False
        msg.manual = False
        msg.manual_gear = 0
        msg.gear_immediate = True
        self.cmd_pub.publish(msg)
        self._cmd_count += 1

    def _advance(self, reason: str) -> None:
        name = self.waypoints[self.idx][2] if self.idx < len(self.waypoints) else "?"
        self.get_logger().info(f"waypoint {name} (idx={self.idx}) {reason}, advancing")
        self.idx += 1

    def _initialize_waypoints_from_gps(self) -> bool:
        with self.lock:
            have_odom = self.have_odom
            have_gps = self.have_gps
            x = self.x
            y = self.y
            gps_lat = self.gps_lat
            gps_lon = self.gps_lon

        if not have_odom or not have_gps:
            return False

        cos0 = math.cos(math.radians(gps_lat))
        local: List[Tuple[float, float, str]] = []
        for lat, lon, name in self.raw_waypoints:
            north = x + (lat - gps_lat) * math.pi / 180.0 * EARTH_RADIUS_M
            east = y + (lon - gps_lon) * math.pi / 180.0 * EARTH_RADIUS_M * cos0
            local.append((north, east, name))

        # 按 CRUISE_ORDER 展开成实际访问序列.
        self.waypoints = [local[i] for i in self.cruise_order]
        self.idx = 0
        self.waypoints_ready = True
        self.get_logger().info(
            f"initialized {len(self.waypoints)} local waypoints from current GPS "
            f"lat={gps_lat:.9f}, lon={gps_lon:.9f}, pose=({x:.2f},{y:.2f})"
        )
        for north, east, name in self.waypoints:
            self.get_logger().info(f"  {name}: north={north:.2f} m, east={east:.2f} m")
        return True

    # ----------------------------------------------------------- main loop

    def _tick(self) -> None:
        self._tick_count += 1
        ticks_per_sec = max(1, int(round(1.0 / self.period)))

        with self.lock:
            have_odom = self.have_odom
            have_gps = self.have_gps
            x, y, yaw = self.x, self.y, self.yaw

        if not have_odom:
            # 没收到 odom 时, 每 ~2s 提醒一次, 别直接静默
            if self._tick_count % (ticks_per_sec * 2) == 1:
                self.get_logger().warn(
                    f"waiting for {self.cmd_pub.topic_name.replace('car_cmd','odom_local')} "
                    f"(no odometry received yet); is agv_imu_odom_node running?"
                )
            return

        if not have_gps:
            if self._tick_count % (ticks_per_sec * 2) == 1:
                self.get_logger().warn(
                    f"waiting for {self.cmd_pub.topic_name.replace('car_cmd','global_gps')} "
                    f"(no GPS received yet); is agv_imu_odom_node running?"
                )
            return

        if not self.waypoints_ready:
            if not self._initialize_waypoints_from_gps():
                return

        if self.idx >= len(self.waypoints):
            # 走完一整圈 (CRUISE_ORDER) 后回到序列开头, 循环往复.
            self.idx = 0

        target_n, target_e, _ = self.waypoints[self.idx]
        dn = target_n - x
        de = target_e - y
        dist = math.hypot(dn, de)
        self._last_dist = dist

        # 到达判据 1: 距离够近
        if dist < self.tol:
            self._advance("within tolerance")
            return

        # 到达判据 2: 已经超过这个航点 (waypoint 在身后) 且距离不大于 2*R_min
        # 用机体朝向投影判断: 投影 < 0 -> 在身后
        forward = dn * math.cos(yaw) + de * math.sin(yaw)
        if dist < 2.0 * self.min_turn_radius and forward < 0.0:
            self._advance("passed (behind heading)")
            return

        # 航向 P 控制
        bearing = math.atan2(de, dn)
        heading_err = wrap_pi(bearing - yaw)
        w_des = self.heading_gain * heading_err

        # 最小转弯半径硬约束: |w| <= v / R_min
        v = self.target_speed
        w_cap_radius = v / self.min_turn_radius
        # 同时不能超过 actor 能输出的物理上限
        w_cap = min(w_cap_radius, self.max_yaw_rate)
        if w_des > w_cap:
            w_des = w_cap
        elif w_des < -w_cap:
            w_des = -w_cap

        # 先原地对准目标, 进入一个很小的容差后再前进, 避免横着切过去.
        if abs(heading_err) > ALIGN_HEADING_TOL:
            throttle = 0.0
        else:
            throttle = v / self.max_speed
        steering = w_des / self.max_yaw_rate
        self._send(throttle, steering, brake=0.0)

        # 每 ~1s 打印一次状态, 便于排查 "小车不动"
        if self._tick_count % ticks_per_sec == 0:
            wp_name = self.waypoints[self.idx][2]
            self.get_logger().info(
                f"target={wp_name}({self.idx}) dist={dist:.2f}m "
                f"hdg_err={math.degrees(heading_err):+.1f}deg "
                f"throttle={throttle:+.2f} steering={steering:+.2f} "
                f"pose=({x:.2f},{y:.2f},{math.degrees(yaw):+.1f}deg) "
                f"cmd_pub#={self._cmd_count}"
            )


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    try:
        node = AgvWaypointCruiseNode()
    except SystemExit as e:
        rclpy.shutdown()
        return int(e.code) if isinstance(e.code, int) else 1
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
