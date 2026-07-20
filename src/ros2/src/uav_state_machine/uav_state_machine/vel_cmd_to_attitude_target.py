import math

from airsim_interfaces.msg import VelCmd
from mavros_msgs.msg import AttitudeTarget
from tf_transformations import quaternion_from_euler

from .pid import PID


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


def _reset_pid(pid: PID, current_value: float) -> None:
    pid.cur_val = current_value
    pid._pre_error = 0.0
    pid._integral = 0.0


def vel_cmd_to_attitude_target(
    vel_cmd: VelCmd,
    current_body_vel,
    stamp,
    pid_x: PID,
    pid_y: PID,
    pid_z: PID,
    # ENU 当前 yaw (rad), 0 = 朝东. 必须与上游计算 cmd vx/vy 时使用的
    # body frame yaw 一致, 否则等于命令飞机先 yaw 回 0 (朝东) 再倾斜,
    # 导致飞行方向相对目标出现 90/180 度系统偏差.
    current_yaw: float = 0.0,
    hover_thrust: float = 0.5,
    # 倾角上限. 调大到 0.5 rad ≈ 28 度, 让 PID 在飞机被吹偏 / 高速惯性
    # 状态时有足够减速能力 (sin28°·g ≈ 4.6 m/s²).
    # 旧值 0.15 在 14 m/s 时只能产生约 1.5 m/s² 减速, 拉不住高速.
    max_tilt: float = 0.5,
    min_thrust: float = 0.05,
    max_thrust: float = 0.90,
) -> AttitudeTarget:
    """Convert a body-frame VelCmd into a MAVROS AttitudeTarget.

    项目整体锚定到 MAVROS ENU/FLU. 上游 ugv_then_uav_node 现在输出的
    VelCmd.linear 就是 FLU 机体系下的期望速度:
      x = forward, y = left, z = up.

    标准 ENU/FLU 姿态映射:
      forward (+x_FLU) -> 机头下俯, pitch < 0
      left    (+y_FLU) -> 向左滚转, roll  < 0
      up      (+z_FLU) -> 加大推力, thrust > hover

    旧实现里的“为兼容 wrapper NWU + AirSim FRD 而做的轴交换 / 翻号”
    已经在上游统一到 ENU/FLU 后失去意义, 这里回到原则化版本.
    """
    des = vel_cmd.twist.linear
    cur = current_body_vel.linear

    # Cap horizontal demand. The original VelCmd values were tuned for direct
    # velocity control and feel too aggressive once mapped into attitude.
    # 把水平速度上限缩到 ±1.5 m/s, 与 max_tilt=0.15 配合避免持续俯冲.
    des_x = _clamp(float(des.x), -1.5, 1.5)
    des_y = _clamp(float(des.y), -1.5, 1.5)
    des_z = float(des.z)
    # 旧实现 (wrapper NWU 时期 z 取反, 整套统一到 ENU/FLU 后不再需要):
    #     des_y = -float(des.y)
    #     des_z = -float(des.z)

    lateral_only_vertical = abs(des_x) < 1e-3 and abs(des_y) < 1e-3

    if lateral_only_vertical:
        # 纯垂直命令: 重置水平 PID 内部状态, 避免历史误差残留导致飞机
        # 在悬停 / 起飞等阶段被误推向某一侧.
        _reset_pid(pid_x, cur.x)
        _reset_pid(pid_y, cur.y)
        pitch = 0.0
        roll = 0.0
    else:
        # 水平闭环 PID: 跟踪 body forward / left 速度, 输出直接作为期望
        # 倾角 (rad). pid.py 是位置式 PID, calculate() 内部用
        # (target - cur_val) 计算误差并把输出存回 cur_val, 所以每次都要
        # 先把 cur_val 设成当前测量再调用 calculate.
        pid_x.target = des_x
        pid_x.cur_val = cur.x
        pitch_pid = pid_x.calculate()

        pid_y.target = des_y
        pid_y.cur_val = cur.y
        roll_pid = pid_y.calculate()

        # ENU/FLU 标准姿态映射:
        #   forward velocity error 正 -> 机头下俯, pitch 取负
        #   left    velocity error 正 -> 向左滚转, roll  取负
        # PID 自身已经按 (max, min) 限幅, 这里再叠 max_tilt 兜底保险.
        pitch = _clamp(-pitch_pid, -max_tilt, max_tilt)
        roll = _clamp(-roll_pid, -max_tilt, max_tilt)

    pid_z.target = des_z
    pid_z.cur_val = cur.z
    thrust = hover_thrust + pid_z.calculate()
    tilt_comp = 1.0 / max(0.5, abs(math.cos(roll) * math.cos(pitch)))
    thrust = _clamp(thrust * tilt_comp, min_thrust, max_thrust)

    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, current_yaw)
    # 旧实现 (yaw 硬编码为 0, 会让 PX4 总是把飞机摆回朝东再倾斜):
    #     qx, qy, qz, qw = quaternion_from_euler(roll, pitch, 0.0)

    msg = AttitudeTarget()
    msg.header.stamp = stamp
    msg.type_mask = (
        AttitudeTarget.IGNORE_ROLL_RATE
        | AttitudeTarget.IGNORE_PITCH_RATE
    )
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    msg.body_rate.x = 0.0
    msg.body_rate.y = 0.0
    msg.body_rate.z = vel_cmd.twist.angular.z
    msg.thrust = thrust
    return msg
