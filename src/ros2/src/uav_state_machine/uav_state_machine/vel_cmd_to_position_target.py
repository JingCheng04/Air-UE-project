"""把上游 FLU body VelCmd 转换为 MAVROS PositionTarget(BODY_NED velocity).

为什么用 PositionTarget 而不是 AttitudeTarget:
- 我们的上游算的是“目标速度”, 不是“目标姿态”.
- PX4 内部有成熟的速度控制环, 用 setpoint_raw/local + FRAME_BODY_NED +
  velocity-only 是 PX4 OFFBOARD 文档推荐的速度控制方式.
- 之前用 AttitudeTarget 时我们在外面再写一层 PID -> 姿态 -> 推力, 既容易
  限幅过小拉不住高速, 又容易限幅过大让飞机俯冲发散; 现在直接把 vx/vy/vz
  交给 PX4 速度环, 这些问题都不需要我们自己解决.

坐标系:
- 上游 VelCmd.linear 是 ROS FLU body (x=forward, y=left, z=up).
- PositionTarget(FRAME_BODY_NED) 在 mavros 内部已经按 ROS baselink (FLU)
  -> aircraft (FRD) 做过坐标转换, 我们这里再手工翻号会变成翻两次,
  方向反向. 实测中: y 不翻号方向正确, z 翻号会让一切到 velocity-only
  立刻坠地, 因此 x/y/z 全部直接透传.

yaw 控制:
- 默认 yaw_setpoint=None: 屏蔽 yaw / yaw_rate, PX4 自己保持当前 yaw.
- 传入 yaw_setpoint (ROS ENU 世界系下的 yaw, rad, +Z 逆时针; yaw=0 朝东):
  启用 yaw 控制, mavros 会把 ENU yaw 映射到 NED yaw 后下发.
  典型用法: 巡航期间让机头持续指向追踪目标, 让前向相机能看到目标.
"""

from airsim_interfaces.msg import VelCmd
from mavros_msgs.msg import PositionTarget


def vel_cmd_to_position_target(
    vel_cmd: VelCmd, stamp, yaw_setpoint: float | None = None,
) -> PositionTarget:
    msg = PositionTarget()
    msg.header.stamp = stamp
    # FRAME_BODY_NED = 8 (mavros_msgs/PositionTarget 常量).
    msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
    # 默认: 只启用 vx/vy/vz, 屏蔽 position / accel / yaw / yaw_rate.
    type_mask = (
        PositionTarget.IGNORE_PX
        | PositionTarget.IGNORE_PY
        | PositionTarget.IGNORE_PZ
        | PositionTarget.IGNORE_AFX
        | PositionTarget.IGNORE_AFY
        | PositionTarget.IGNORE_AFZ
        | PositionTarget.IGNORE_YAW
        | PositionTarget.IGNORE_YAW_RATE
    )

    # mavros 已做 FLU->FRD 转换, 这里全部直接透传, 不再手工翻号.
    msg.velocity.x = float(vel_cmd.twist.linear.x)
    msg.velocity.y = float(vel_cmd.twist.linear.y)
    msg.velocity.z = float(vel_cmd.twist.linear.z)
    # 旧实现 (手工 z 翻号, 会让 z 被翻两次, 切到 velocity-only 立刻坠地):
    #     msg.velocity.z = -float(vel_cmd.twist.linear.z)

    # 可选 yaw 控制. 传入 None 时保持原行为 (PX4 锁当前 yaw).
    # 传入数值时启用 yaw 控制, 关闭 IGNORE_YAW, 保持 IGNORE_YAW_RATE.
    if yaw_setpoint is not None:
        type_mask &= ~PositionTarget.IGNORE_YAW
        msg.yaw = float(yaw_setpoint)

    msg.type_mask = type_mask
    return msg
