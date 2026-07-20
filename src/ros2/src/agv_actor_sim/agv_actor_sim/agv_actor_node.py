"""伪 AGV 运动学控制节点 (基于 UE actor + simSetObjectPose).

订阅 ROS2 topic `car_cmd` (airsim_interfaces/msg/CarControls), 用与 Cosys-AirSim
真实 wrapper 同样的字段语义把 throttle/steering/brake 解释成差速车的速度和角速度,
再周期性调用 AirSim Python API simSetObjectPose 更新 UE 关卡里 actor (默认
BP_HuskyVisual_C_1) 的位姿.

控制模型 (skid-steer 简化):
    线速度 v        = clamp(throttle, -1, 1) * max_speed
                       brake>0 时按比例衰减
    机体角速度 yaw_rate = clamp(steering, -1, 1) * max_yaw_rate

每个 tick 用最近一次的 (v, yaw_rate) 做欧拉积分, 限制 yaw_rate 上限避免
下游伪 IMU 微分发散.

Topic / 参数完全可配置, 默认值与项目里 src/test/joint/ 的脚本风格一致.
"""

from __future__ import annotations

import math
import threading
import time

import cosysairsim as airsim
import rclpy
from airsim_interfaces.msg import CarControls
from rclpy.node import Node


def yaw_to_quat(yaw_rad: float) -> "airsim.Quaternionr":
    """偏航角 -> AirSim 四元数."""
    half = yaw_rad * 0.5
    return airsim.Quaternionr(
        x_val=0.0,
        y_val=0.0,
        z_val=math.sin(half),
        w_val=math.cos(half),
    )


def quat_to_yaw(q: "airsim.Quaternionr") -> float:
    """从 AirSim 四元数提取偏航角."""
    siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
    cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
    return math.atan2(siny, cosy)


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AgvActorNode(Node):
    """订阅 car_cmd, 周期推动 UE actor 的运动学控制节点."""

    def __init__(self) -> None:
        super().__init__("agv_actor_node")

        # 节点参数 (运行时可改, 例如 ros2 run ... --ros-args -p object:=Foo)
        self.declare_parameter("object", "BP_HuskyVisual_C_1")
        self.declare_parameter("topic_prefix", "/sim_ugv/airsim_node/UGV_1")
        self.declare_parameter("host_ip", "127.0.0.1")
        self.declare_parameter("host_port", 41451)
        self.declare_parameter("max_speed", 2.0)         # m/s, throttle=1 对应的目标线速度
        self.declare_parameter("max_yaw_rate", 90.0)     # deg/s, steering=±1 对应的目标角速度
        self.declare_parameter("rate", 60.0)             # Hz, 控制循环频率

        self.object_name = str(self.get_parameter("object").value)
        prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        host_ip = str(self.get_parameter("host_ip").value)
        host_port = int(self.get_parameter("host_port").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_yaw_rate = math.radians(float(self.get_parameter("max_yaw_rate").value))
        rate_hz = float(self.get_parameter("rate").value)
        self.period = 1.0 / max(1.0, rate_hz)

        # AirSim 客户端: 我们只用它做 actor pose 读写, 不接管车辆控制
        self.client = airsim.MultirotorClient(ip=host_ip, port=host_port)
        self.client.confirmConnection()

        # 最近一次收到的指令; 默认全零, 即静止
        self._lock = threading.Lock()
        self._throttle = 0.0
        self._steering = 0.0
        self._brake = 0.0
        self._reverse = False

        # actor 当前位姿状态; 初始化时从 AirSim 读一次
        pose = self.client.simGetObjectPose(self.object_name)
        if not math.isfinite(pose.position.x_val):
            self.get_logger().error(
                f"actor '{self.object_name}' pose is NaN; check World Outliner name and re-Play"
            )
            raise SystemExit(2)
        self._x = float(pose.position.x_val)
        self._y = float(pose.position.y_val)
        self._z = float(pose.position.z_val)
        self._yaw = quat_to_yaw(pose.orientation)
        if not math.isfinite(self._yaw):
            self._yaw = 0.0

        # 订阅 car_cmd
        cmd_topic = f"{prefix}/car_cmd"
        self.create_subscription(CarControls, cmd_topic, self._on_cmd, 10)
        self.get_logger().info(
            f"object='{self.object_name}' subscribing {cmd_topic}; "
            f"host={host_ip}:{host_port}; "
            f"max_speed={self.max_speed:.2f} m/s, "
            f"max_yaw_rate={math.degrees(self.max_yaw_rate):.1f} deg/s"
        )

        # 周期控制循环
        self.create_timer(self.period, self._tick)

    def _on_cmd(self, msg: CarControls) -> None:
        with self._lock:
            self._throttle = max(-1.0, min(1.0, float(msg.throttle)))
            self._steering = max(-1.0, min(1.0, float(msg.steering)))
            self._brake = max(0.0, min(1.0, float(msg.brake)))
            self._reverse = bool(getattr(msg, "manual", False)) and (msg.manual_gear < 0)

    def _tick(self) -> None:
        # 读出当前指令
        with self._lock:
            throttle = self._throttle
            steering = self._steering
            brake = self._brake

        # brake 越大, 实际线速度越接近 0
        effective_throttle = throttle * (1.0 - brake)
        v = effective_throttle * self.max_speed
        yaw_rate = steering * self.max_yaw_rate

        # 欧拉积分一步
        dt = self.period
        self._yaw = wrap_pi(self._yaw + yaw_rate * dt)
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        # 写回 UE
        pose = airsim.Pose(
            airsim.Vector3r(self._x, self._y, self._z),
            yaw_to_quat(self._yaw),
        )
        try:
            self.client.simSetObjectPose(self.object_name, pose, teleport=True)
        except Exception as e:
            self.get_logger().warn(f"simSetObjectPose error: {e}")


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    try:
        node = AgvActorNode()
    except SystemExit as e:
        rclpy.shutdown()
        return int(e.code) if isinstance(e.code, int) else 1
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
