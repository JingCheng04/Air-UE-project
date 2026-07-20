"""伪 AGV IMU + Odometry 发布节点 (基于 simGetObjectPose 反推).

周期采样 UE 关卡里 actor (默认 BP_HuskyVisual_C_1) 的位姿, 做差分还原:
    angular_velocity   (机体系, 由四元数微分)
    linear_acceleration (机体系比力, 包含重力, 与真 IMU 一致)
    odometry            (位姿 + 一阶差分速度)

Topic 命名沿用 Cosys-AirSim wrapper 风格, 通过 topic_prefix 参数与真 wrapper 区分:
    <prefix>/imu/<imu_name>    sensor_msgs/Imu
    <prefix>/odom_local        nav_msgs/Odometry

设计上与 src/test/joint/ugv_pseudo_imu.py 一致, 这里把它整理进 ROS2 ament_python
package, 便于 launch 启动 + colcon build 管理.

额外发布:
    <prefix>/global_gps      sensor_msgs/NavSatFix
    <prefix>/gps/<gps_name>  sensor_msgs/NavSatFix

GPS 话题格式与 UAV 的 /global_gps 一致, 基于 AirSim / UE 世界 NED 坐标和
OriginGeopoint 做局部平面近似换算. 为了与 AirSim UAV 原生 GPS 完全对齐,
在这条 "UE 世界坐标 -> 原始 GPS" 转换后, 还会叠加一个一次性校准得到的
固定偏移量 (bias). 不修改任何 UE 元素.
"""

from __future__ import annotations

import math
from collections import deque

import cosysairsim as airsim
import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix


# AirSim NED: +z 朝下, 重力沿 +z
GRAVITY_NED = (0.0, 0.0, 9.81)
EARTH_RADIUS_M = 6378137.0


def quat_to_R(qx: float, qy: float, qz: float, qw: float):
    """四元数 -> 机体到世界系的 3x3 旋转矩阵 (嵌套元组)."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)),
        (2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)),
    )


def matT_mul_vec(R, v):
    """计算 R^T * v, 把世界系向量映射回机体系."""
    return (
        R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
        R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
        R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2],
    )


def quat_relative(q1, q0):
    """q_rel = q1 * conj(q0); 输入/输出都是 (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x0, y0, z0, w0 = q0
    cx, cy, cz, cw = -x0, -y0, -z0, w0
    return (
        w1 * cx + x1 * cw + y1 * cz - z1 * cy,
        w1 * cy - x1 * cz + y1 * cw + z1 * cx,
        w1 * cz + x1 * cy - y1 * cx + z1 * cw,
        w1 * cw - x1 * cx - y1 * cy - z1 * cz,
    )


class AgvImuOdomNode(Node):
    """周期采样 UE actor 位姿, 反推并发布 IMU + Odometry."""

    def __init__(self) -> None:
        super().__init__("agv_imu_odom_node")

        # 节点参数, 命令行可覆盖
        self.declare_parameter("object", "BP_HuskyVisual_C_1")
        self.declare_parameter("topic_prefix", "/sim_ugv/airsim_node/UGV_1")
        self.declare_parameter("vehicle", "UGV_1")     # 仅用于 frame_id 命名
        self.declare_parameter("imu_name", "UGV_1_Imu")
        self.declare_parameter("gps_name", "UGV_1_Gps")
        self.declare_parameter("align_uav_name", "UAV_1")
        self.declare_parameter("align_uav_gps_name", "UAV_1_Gps")
        self.declare_parameter("host_ip", "127.0.0.1")
        self.declare_parameter("host_port", 41451)
        self.declare_parameter("rate", 30.0)
        # Default to the same geodetic origin as the UAV AirSim settings.
        self.declare_parameter("origin_latitude", 45.72060377096292)
        self.declare_parameter("origin_longitude", -123.93305245338378)
        self.declare_parameter("origin_altitude", 0.0)

        self.object_name = str(self.get_parameter("object").value)
        prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        vehicle = str(self.get_parameter("vehicle").value)
        imu_name = str(self.get_parameter("imu_name").value)
        gps_name = str(self.get_parameter("gps_name").value)
        self.align_uav_name = str(self.get_parameter("align_uav_name").value)
        self.align_uav_gps_name = str(self.get_parameter("align_uav_gps_name").value)
        host_ip = str(self.get_parameter("host_ip").value)
        host_port = int(self.get_parameter("host_port").value)
        rate = max(1.0, min(30.0, float(self.get_parameter("rate").value)))
        self.period = 1.0 / rate
        self.origin_lat = float(self.get_parameter("origin_latitude").value)
        self.origin_lon = float(self.get_parameter("origin_longitude").value)
        self.origin_alt = float(self.get_parameter("origin_altitude").value)

        # AirSim 客户端只读 actor 位姿
        self.client = airsim.MultirotorClient(ip=host_ip, port=host_port)
        self.client.confirmConnection()
        self._nan_warned = False
        self.gps_bias_lat = 0.0
        self.gps_bias_lon = 0.0
        self.gps_bias_alt = 0.0

        # Topic
        imu_topic = f"{prefix}/imu/{imu_name}"
        odom_topic = f"{prefix}/odom_local"
        gps_topic = f"{prefix}/global_gps"
        gps_sensor_topic = f"{prefix}/gps/{gps_name}"
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.imu_pub = self.create_publisher(Imu, imu_topic, qos)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, qos)
        self.gps_pub = self.create_publisher(NavSatFix, gps_topic, qos)
        self.gps_sensor_pub = self.create_publisher(NavSatFix, gps_sensor_topic, qos)

        # 滚动缓存: (t_ns, x, y, z, qx, qy, qz, qw)
        self.buf: deque = deque(maxlen=3)

        # frame_id 约定
        self.imu_frame = f"{vehicle}/{imu_name}"
        self.odom_frame = f"{vehicle}/odom_local"
        self.body_frame = vehicle

        # 用当前 AGV 位置与 AirSim UAV 原生 GPS 做一次静态对齐, 之后 AGV GPS
        # 始终发布为: 原始换算值 + AirSim GPS 偏移量.
        self._calibrate_gps_bias()

        self.get_logger().info(
            f"sampling object='{self.object_name}' at {rate:.1f} Hz; "
            f"host={host_ip}:{host_port}; "
            f"publishing {imu_topic}, {odom_topic}, {gps_topic} and {gps_sensor_topic}"
        )

        self.create_timer(self.period, self._tick)

    def _raw_gps_from_ned(self, x_north: float, y_east: float, z_down: float) -> tuple[float, float, float]:
        """世界 NED -> 基于 OriginGeopoint 的原始 GPS (未做 AirSim bias 对齐)."""
        lat0_rad = math.radians(self.origin_lat)
        lat = self.origin_lat + (x_north / EARTH_RADIUS_M) * (180.0 / math.pi)
        lon = self.origin_lon + (y_east / (EARTH_RADIUS_M * math.cos(lat0_rad))) * (180.0 / math.pi)
        alt = self.origin_alt - z_down
        return lat, lon, alt

    def _calibrate_gps_bias(self) -> None:
        """用 AirSim UAV 原生 GPS 与当前 AGV 原始换算 GPS 做一次偏移校准."""
        try:
            pose = self.client.simGetObjectPose(self.object_name)
            x = float(pose.position.x_val)
            y = float(pose.position.y_val)
            z = float(pose.position.z_val)
            raw_lat, raw_lon, raw_alt = self._raw_gps_from_ned(x, y, z)

            gps_data = self.client.getGpsData(
                gps_name=self.align_uav_gps_name,
                vehicle_name=self.align_uav_name,
            )
            uav_lat = float(gps_data.gnss.geo_point.latitude)
            uav_lon = float(gps_data.gnss.geo_point.longitude)
            uav_alt = float(gps_data.gnss.geo_point.altitude)

            if not all(map(math.isfinite, (raw_lat, raw_lon, raw_alt, uav_lat, uav_lon, uav_alt))):
                self.get_logger().warn("GPS bias calibration skipped: non-finite AGV/UAV GPS value")
                return

            self.gps_bias_lat = uav_lat - raw_lat
            self.gps_bias_lon = uav_lon - raw_lon
            self.gps_bias_alt = uav_alt - raw_alt
            self.get_logger().info(
                f"GPS bias calibrated against {self.align_uav_name}: "
                f"dlat={self.gps_bias_lat:.9f}, dlon={self.gps_bias_lon:.9f}, dalt={self.gps_bias_alt:.3f}"
            )
        except Exception as e:
            self.get_logger().warn(f"GPS bias calibration skipped: {e}")

    def _tick(self) -> None:
        # 1. 取一次位姿
        try:
            pose = self.client.simGetObjectPose(self.object_name)
        except Exception as e:
            self.get_logger().warn(f"simGetObjectPose error: {e}")
            return

        p = pose.position
        q = pose.orientation
        x, y, z = float(p.x_val), float(p.y_val), float(p.z_val)
        qx, qy, qz, qw = float(q.x_val), float(q.y_val), float(q.z_val), float(q.w_val)

        # 2. NaN 防护
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
                and math.isfinite(qx) and math.isfinite(qy) and math.isfinite(qz) and math.isfinite(qw)):
            if not self._nan_warned:
                self.get_logger().warn(
                    f"actor '{self.object_name}' returned NaN; check World Outliner name"
                )
                self._nan_warned = True
            return
        self._nan_warned = False

        # 3. 进入滚动缓存
        t_ns = self.get_clock().now().nanoseconds
        self.buf.append((t_ns, x, y, z, qx, qy, qz, qw))
        if len(self.buf) < 2:
            return

        # 4. 一阶差分: 平动速度 + 角速度
        t1, x1, y1, z1, qx1, qy1, qz1, qw1 = self.buf[-1]
        t0, x0, y0, z0, qx0, qy0, qz0, qw0 = self.buf[-2]
        dt = max(1e-6, (t1 - t0) * 1e-9)
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        vz = (z1 - z0) / dt
        rel = quat_relative((qx1, qy1, qz1, qw1), (qx0, qy0, qz0, qw0))
        wx_b = 2.0 * rel[0] / dt
        wy_b = 2.0 * rel[1] / dt
        wz_b = 2.0 * rel[2] / dt

        # 5. 二阶中心差分得到世界系加速度, 转成机体比力
        if len(self.buf) >= 3:
            t2 = self.buf[-1][0]
            t0b = self.buf[-3][0]
            dt_a = max(1e-6, (t2 - t0b) * 1e-9 * 0.5)
            ax_w = (self.buf[-1][1] - 2 * self.buf[-2][1] + self.buf[-3][1]) / (dt_a * dt_a)
            ay_w = (self.buf[-1][2] - 2 * self.buf[-2][2] + self.buf[-3][2]) / (dt_a * dt_a)
            az_w = (self.buf[-1][3] - 2 * self.buf[-2][3] + self.buf[-3][3]) / (dt_a * dt_a)
        else:
            ax_w = ay_w = az_w = 0.0

        sf_w = (ax_w - GRAVITY_NED[0], ay_w - GRAVITY_NED[1], az_w - GRAVITY_NED[2])
        R = quat_to_R(qx1, qy1, qz1, qw1)
        sf_b = matT_mul_vec(R, sf_w)

        stamp = self.get_clock().now().to_msg()

        # 6. 发布 Imu
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation = Quaternion(x=qx1, y=qy1, z=qz1, w=qw1)
        imu.angular_velocity = Vector3(x=wx_b, y=wy_b, z=wz_b)
        imu.linear_acceleration = Vector3(x=sf_b[0], y=sf_b[1], z=sf_b[2])
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        # 7. 发布 Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = x1
        odom.pose.pose.position.y = y1
        odom.pose.pose.position.z = z1
        odom.pose.pose.orientation = Quaternion(x=qx1, y=qy1, z=qz1, w=qw1)
        odom.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)
        odom.twist.twist.angular = Vector3(x=wx_b, y=wy_b, z=wz_b)
        self.odom_pub.publish(odom)

        # 8. 发布与 UAV 同格式的 global_gps
        #
        # AirSim / UE 里 actor 的 pose 这里按局部 NED 解释:
        #   x -> North (m)
        #   y -> East  (m)
        #   z -> Down  (m)
        #
        # 与 AirSim UAV 的 global_gps 保持同一地理基准, 用一个小范围平面近似把
        # NED 平移量换成经纬度增量:
        #   latitude  = origin_lat + north / R
        #   longitude = origin_lon + east / (R * cos(origin_lat))
        #   altitude  = origin_alt - down
        #
        # 第一步: UE / AirSim 世界坐标 -> 原始 GPS
        # 第二步: 再叠加与 AirSim UAV 原生 GPS 对齐得到的固定 bias
        # 这样 AGV 的 global_gps 与 UAV 的 global_gps 保持同一参考系。
        lat, lon, alt = self._raw_gps_from_ned(x1, y1, z1)
        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.header.frame_id = self.body_frame
        gps.status.status = 3
        gps.status.service = 2
        gps.latitude = lat + self.gps_bias_lat
        gps.longitude = lon + self.gps_bias_lon
        gps.altitude = alt + self.gps_bias_alt
        self.gps_pub.publish(gps)
        self.gps_sensor_pub.publish(gps)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = AgvImuOdomNode()
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
