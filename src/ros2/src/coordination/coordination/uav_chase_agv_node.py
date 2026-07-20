"""无人机追踪 AGV (按 GPS) 并在指定距离对地悬停.

任务流程:
1. 节点构造期      - 立即向 follow 节点发 ``recovery_cmd=bind``, 让 UAV 在
                     启动瞬间就被钉到 AGV 上, 跟随 AGV 一起运动 (无相对位移)
2. WAIT_READY      - 等 UAV/UGV odom + GPS 就绪 (父类阶段)
3. WAIT_START_DELAY - 父类等待 ``ugv_start_delay`` 秒, 这里直接复用为
                     "AGV 起步预热 5s" 的阶段, 期间 UAV 仍被 follow 钉在车上
4. UAV_TAKEOFF     - 父类的 ``_release_uav()`` 自动发 ``release``, 解除绑定
                     再调 takeoff 服务
5. UAV_ASCEND      - 爬升到相对起飞点 +``uav_height`` (默认 8m)
6. CHASE_AGV       - 持续读 AGV ``/global_gps``, 把 target_lat/lon 切到
                     AGV 当前位姿, 复用父类巡航 + 避障 (DWB + APF)
                     距离 ≤ ``keep_distance`` (默认 15m) 时切到 HOVER
                     若距离再次 > keep_distance + hold_band, 自动回到 CHASE
7. HOVER_OVER_AGV  - 0 水平速度 + 高度环锁定, 不下降

复用 ``UgvThenUavNode`` 的全部算法和参数: 地球-flat GPS 偏差换算、cruise vel
合成、DWB / APF 避障、距离限速安全网、高度环.

机头对准:
    在 CHASE_AGV / HOVER_OVER_AGV 期间, 持续向 ``/uav/control/yaw_setpoint``
    发布指向 AGV 的 yaw (ENU 世界系, rad). uav_state_machine_node 会把这个
    yaw 透传到 PositionTarget 的 yaw 字段, PX4 自动旋转机头. 这样前向相机
    始终看到 AGV, 而不是只做侧向平移.

不修改 agv_imu_odom_node.py: 仅订阅它已有的 GPS 话题
``<agv_prefix>/global_gps`` (默认 ``/sim_ugv/airsim_node/UGV_1/global_gps``).
绑定/解绑通过 ``uav_follow_agv_node`` 实现 (它需要被 launch 同时拉起).
"""
from __future__ import annotations

import math
import time

import rclpy
from airsim_interfaces.msg import VelCmd
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

from coordination.ugv_then_uav_node import (
    STATE_AVOID_OBSTACLE,
    STATE_GO_TO_TARGET,
    UgvThenUavNode,
)


# 与 uav_state_machine_node 中的常量保持一致.
YAW_SETPOINT_TOPIC = "/uav/control/yaw_setpoint"
# yaw 死区: UAV 到 AGV 的水平距离小于此阈值时不再更新 yaw, 避免在飞机
# 几乎正对 AGV 时因 GPS 噪声导致 atan2 跳变, 机头疯转.
YAW_UPDATE_MIN_DIST_M = 0.5


class UavChaseAgvNode(UgvThenUavNode):
    def __init__(self) -> None:
        super().__init__()

        # ---- 新增参数 ----
        self.declare_parameter("agv_prefix", "/sim_ugv/airsim_node/UGV_1")
        # 与 AGV 保持的水平距离 (m). 到达后悬停, 不再前进.
        self.declare_parameter("keep_distance", 15.0)
        # 防抖死区: 距离掉入 (keep_distance, keep_distance+hold_band) 之间时
        # 保持当前状态不切换.
        self.declare_parameter("hold_band", 1.5)
        # 启动后是否立即向 follow 节点发 bind. 默认 True.
        # 注意: 必须同时把 uav_follow_agv_node 拉起来, 否则 bind 命令没人执行.
        self.declare_parameter("bind_on_startup", True)
        # 在 bind 状态下重发 bind 命令的频率 (Hz). 部分情况下 follow 节点起得
        # 比本节点晚, 单帧发 bind 会丢; 因此进入 bind 状态后持续以低速率重发,
        # 直到状态机离开 WAIT_START_DELAY.
        self.declare_parameter("bind_resend_rate", 5.0)
        # release -> takeoff 之间的强制间隔 (s). 关键不变量:
        # follow_agv_node 默认 60 Hz simSetKinematics, "release" 命令到 follow
        # 节点至少要一个 tick (~16.7 ms) 才能真正停止. 父类 _release_uav 与
        # takeoff RPC 在同一 tick 触发, 两个 RPC 撞在一起会让 airsim_node 抛
        # rpc::rpc_error 直接崩. 所以本子类在进入 UAV_TAKEOFF 后先发 release,
        # 静默等 release_grace_seconds 再让父类发 takeoff.
        # 默认 0.3s 远超 33ms 的最小要求, 兼顾 RPC 排队和 AirSim 内部状态切换.
        self.declare_parameter("release_grace_seconds", 0.3)

        agv_prefix = str(self.get_parameter("agv_prefix").value).rstrip("/")
        self.keep_distance = float(self.get_parameter("keep_distance").value)
        self.hold_band = float(self.get_parameter("hold_band").value)
        self._bind_on_startup = bool(self.get_parameter("bind_on_startup").value)
        bind_resend_rate = float(self.get_parameter("bind_resend_rate").value)
        self._release_grace = float(self.get_parameter("release_grace_seconds").value)
        # release_grace 阶段的内部计时器, 进入 UAV_TAKEOFF 才被设置.
        self._release_grace_until: float | None = None

        # 父类已经声明 ugv_distance / ugv_start_delay, 这里不再重复 declare,
        # 仅在运行期覆盖语义: ugv_distance=0 跳过 UGV_FORWARD, ugv_start_delay
        # 直接当作 "AGV 起步 + UAV 跟车 5s 预热" 时长.
        # launch 文件已经把这两个参数传成期望值; 这里再保险一遍.
        if self.ugv_distance != 0.0:
            self.ugv_distance = 0.0

        # ---- AGV GPS 订阅 ----
        # 与 UAV 的 /global_gps 同类型, 由 agv_imu_odom_node 发布.
        self._agv_lat: float | None = None
        self._agv_lon: float | None = None
        self.create_subscription(
            NavSatFix, f"{agv_prefix}/global_gps",
            self._agv_gps_cb, qos_profile_sensor_data,
        )

        # ---- yaw setpoint publisher ----
        # 仅在 CHASE_AGV / HOVER_OVER_AGV 阶段发布, 让 uav_state_machine_node
        # 把指向 AGV 的 yaw 透传给 PX4. ENU 世界系下 yaw=0 朝东, +Z 逆时针.
        self.yaw_setpoint_pub = self.create_publisher(
            Float32, YAW_SETPOINT_TOPIC, 10,
        )
        # 缓存上一次发布的 yaw, 配合死区去抖.
        self._last_yaw_cmd: float | None = None

        # ---- 启动期立刻 bind, 让 UAV 与 AGV 同步起步 ----
        # 注意: 父类的 recovery_cmd_pub 已经创建好, 直接复用.
        # 状态: 'bind' 持续重发到离开 WAIT_START_DELAY 为止;
        # 一旦进入 UAV_TAKEOFF, 父类 _release_uav 会发 release, 此后停发.
        self._bind_active = self._bind_on_startup
        if self._bind_on_startup:
            self._send_recovery_cmd("bind")
            # 周期重发, 容忍 follow 节点晚启动.
            period = max(0.05, 1.0 / max(bind_resend_rate, 0.1))
            self.create_timer(period, self._bind_resend_tick)

        self.get_logger().info(
            f"chase config: agv_gps={agv_prefix}/global_gps, "
            f"warmup={self.ugv_start_delay:.1f}s (=ugv_start_delay), "
            f"keep_distance={self.keep_distance:.2f}m, hold_band={self.hold_band:.2f}m, "
            f"bind_on_startup={self._bind_on_startup}"
        )

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _agv_gps_cb(self, msg: NavSatFix) -> None:
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        self._agv_lat = float(msg.latitude)
        self._agv_lon = float(msg.longitude)

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def _send_recovery_cmd(self, value: str) -> None:
        msg = String()
        msg.data = value
        self.recovery_cmd_pub.publish(msg)

    def _bind_resend_tick(self) -> None:
        # 在 WAIT_READY / WAIT_START_DELAY 期间持续重发 bind. 一旦进入
        # UAV_TAKEOFF, 父类会发 release, 我们停发.
        if not self._bind_active:
            return
        if self.state in ("WAIT_READY", "WAIT_START_DELAY", "UGV_FORWARD"):
            self._send_recovery_cmd("bind")
        else:
            # 已经离开起步阶段, 不再发 bind. 之后绑定逻辑由 release 接管.
            self._bind_active = False

    def _have_agv_gps(self) -> bool:
        return self._agv_lat is not None and self._agv_lon is not None

    def _xy_distance_to_agv(self) -> float:
        """大圆近似下 UAV 到 AGV 的水平距离 (m). 与父类 _target_delta_m 一致."""
        assert self._agv_lat is not None and self._agv_lon is not None
        EARTH_RADIUS_M = 6378137.0
        lat = math.radians(self.uav_lat)
        north = math.radians(self._agv_lat - self.uav_lat) * EARTH_RADIUS_M
        east = math.radians(self._agv_lon - self.uav_lon) * EARTH_RADIUS_M * math.cos(lat)
        return math.hypot(north, east)

    def _publish_hover_cmd(self) -> None:
        """对地悬停: 水平速度 0, 高度由父类 _height_vz 锁定."""
        cmd = VelCmd()
        cmd.twist.linear.z = self._height_vz()
        # 仍然走 GO_TO_TARGET 候选指令, 让状态机选择转发.
        self.target_cmd_pub.publish(cmd)

    def _publish_yaw_to_agv(self) -> None:
        """以 UAV 当前位置为原点, 算指向 AGV 的 ENU yaw 并发布.

        坐标系: 与 _publish_dwb_goal_only 一致, 用 (east, north) 计算
        ``atan2(north, east)``. ROS ENU 下 yaw=0 朝东, +Z 逆时针, 所以这就是
        让机头朝向 AGV 的世界 yaw. uav_state_machine_node 会在 GO_TO_TARGET
        状态下把它透传给 PX4 的 PositionTarget.yaw.

        实测发现需要加 π (180度) 才能让机头对准 AGV, 否则是机尾朝向 AGV.
        可能是 MAVROS 或 PX4 在某个环节对 yaw 做了额外转换.

        距离过近时跳过更新, 避免 atan2 在 GPS 噪声下抖动让机头疯转;
        没有 AGV GPS 时也跳过.
        """
        if not self._have_agv_gps():
            return
        # _target_delta_m 已经按 self.target_lat/lon 在 _chase_step / _hover_step
        # 里被设成 AGV 当前位置, 直接复用即可保证 yaw 与水平速度同源.
        east, north = self._target_delta_m()
        if math.hypot(east, north) < YAW_UPDATE_MIN_DIST_M:
            # 死区: 保持上一次 yaw, 防 GPS 噪声让机头疯转.
            if self._last_yaw_cmd is None:
                return
            yaw = self._last_yaw_cmd
        else:
            # 实测: 需要加 π 才能让机头对准 AGV (否则机尾朝向 AGV).
            yaw = math.atan2(north, east) + math.pi
        self._last_yaw_cmd = yaw
        msg = Float32()
        msg.data = float(yaw)
        self.yaw_setpoint_pub.publish(msg)

    # ------------------------------------------------------------------
    # 主状态机重写
    # ------------------------------------------------------------------
    def _tick(self) -> None:  # noqa: C901
        # 父类的 WAIT_READY / WAIT_START_DELAY / UAV_TAKEOFF / UAV_ASCEND
        # 完全沿用 (UGV_FORWARD 因为 ugv_distance=0 自动跳过).
        # 重要: WAIT_START_DELAY 期间 UAV 已被 follow 节点钉在 AGV 上, 因此即使
        # AGV 已经在沿 waypoint 巡航, UAV 也在车上同步移动, 不需要主动控制.
        # 父类 _tick 在 UAV_ASCEND 完成时切到 FLY_TO_TARGET, 我们改道到 CHASE_AGV.

        # ---- UAV_TAKEOFF: 插入 release grace, 防止 release 与 takeoff RPC 撞车 ----
        # 父类原始逻辑: 同一 tick 里发 release + 调 takeoff 服务 -> 与 follow 节点
        # 还在跑的 simSetKinematics 在 AirSim RPC 队列上撞, server 抛 rpc_error
        # 然后 airsim_node 进程 abort. 这里的兜底:
        #   1) 第一次进入 UAV_TAKEOFF: 先发 release (走父类 _release_uav), 记录
        #      grace 截止时间, 当帧不调 takeoff
        #   2) 在 grace 内每 tick 不动作 (follow 节点已经在自己的 tick 解绑)
        #   3) grace 过去后才放行父类 _tick, 让它真正发 takeoff 服务请求
        if self.state == "UAV_TAKEOFF" and not self.takeoff_requested:
            if self._release_grace_until is None:
                # 触发 release. 父类 _release_uav 内部对重复发 release 是幂等的.
                self._release_uav()
                # 同步关掉本节点的 bind 重发, 否则 release / bind 互相覆盖.
                self._bind_active = False
                self._release_grace_until = time.time() + self._release_grace
                self.get_logger().info(
                    f"release sent; waiting {self._release_grace:.2f}s before takeoff "
                    f"to let follow_agv_node unbind cleanly"
                )
                return
            if time.time() < self._release_grace_until:
                # grace 内: 跳过父类 _tick, 防止它在 follow 节点解绑前调 takeoff.
                return
            # grace 过去: 落到下面的 super()._tick() 进入正常 takeoff 流程.

        if self.state in (
            "WAIT_READY",
            "WAIT_START_DELAY",
            "UGV_FORWARD",
            "UAV_TAKEOFF",
            "UAV_ASCEND",
        ):
            super()._tick()
            return

        if self.state == "FLY_TO_TARGET":
            self.state = "CHASE_AGV"
            self.get_logger().info("ASCEND complete; entering CHASE_AGV")

        if self.state == "CHASE_AGV":
            self._chase_step()
            return

        if self.state == "HOVER_OVER_AGV":
            self._hover_step()
            return

        # 其他状态 (LANDING / DONE) 不应被进入, 兜底交给父类.
        super()._tick()

    # ------------------------------------------------------------------
    # 子阶段
    # ------------------------------------------------------------------
    def _chase_step(self) -> None:
        if not self._have_agv_gps():
            self.get_logger().warn(
                "AGV GPS not received yet; hovering",
                throttle_duration_sec=2.0,
            )
            self._publish_hover_cmd()
            self._publish_state(STATE_GO_TO_TARGET)
            return

        # 把巡航目标改为 AGV 当前 GPS. 父类 _publish_target_cmd / 避障逻辑都用
        # self.target_lat/lon, 直接覆盖即可.
        assert self._agv_lat is not None and self._agv_lon is not None
        self.target_lat = self._agv_lat
        self.target_lon = self._agv_lon

        # 持续发布 yaw setpoint 让机头指向 AGV. 这一帧无论后面走 cruise / avoid /
        # hover 哪条分支, 状态机都在 GO_TO_TARGET 下消费它.
        self._publish_yaw_to_agv()

        # 障碍迟滞 (与父类 FLY_TO_TARGET 中一致). 安全优先级最高.
        now = time.time()
        obstacle_seen = self.min_scan_range <= self.obstacle_distance_limit
        if not self.avoid_active:
            if obstacle_seen:
                self.avoid_active = True
                self.avoid_clear_since = None
        else:
            if self.min_scan_range >= self.obstacle_clear_distance:
                if self.avoid_clear_since is None:
                    self.avoid_clear_since = now
                elif (now - self.avoid_clear_since) >= self.obstacle_clear_hold:
                    self.avoid_active = False
                    self.avoid_clear_since = None
            else:
                self.avoid_clear_since = None

        if self.avoid_active:
            self._publish_state(STATE_AVOID_OBSTACLE)
            self._publish_dwb_goal_only()
            return

        # 距离判定: 进入 keep_distance 内悬停.
        dist = self._xy_distance_to_agv()
        if dist <= self.keep_distance:
            self.get_logger().info(
                f"AGV reached within {dist:.2f}m (<= keep_distance={self.keep_distance:.2f}m); "
                f"switching to HOVER_OVER_AGV"
            )
            self.state = "HOVER_OVER_AGV"
            self._publish_hover_cmd()
            self._publish_state(STATE_GO_TO_TARGET)
            return

        # 正常追踪: 调用父类的 cruise + DWB-goal 发布. 父类内部对短距离会
        # 自动减速 (speed = min(cruise_speed, 0.8 * dist)), 不会冲过头.
        self._publish_target_cmd()
        self._publish_state(STATE_GO_TO_TARGET)

    def _hover_step(self) -> None:
        # 悬停期间也要保持 target_lat/lon = AGV (chase->hover 切换时是上一帧
        # 设的, 但 AGV 在动, 不更新会让 yaw 对不上). 这里直接同步.
        if self._have_agv_gps():
            assert self._agv_lat is not None and self._agv_lon is not None
            self.target_lat = self._agv_lat
            self.target_lon = self._agv_lon
            dist = self._xy_distance_to_agv()
            if dist > (self.keep_distance + self.hold_band):
                self.get_logger().info(
                    f"AGV moved away to {dist:.2f}m; resume CHASE_AGV"
                )
                self.state = "CHASE_AGV"
                return
        # 悬停期间也持续指向 AGV, 让相机一直看到目标.
        self._publish_yaw_to_agv()
        self._publish_hover_cmd()
        self._publish_state(STATE_GO_TO_TARGET)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = UavChaseAgvNode()
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
