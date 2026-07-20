# ROS 2 工作区
coordination 包的节点、话题和三个协同 demo 详见：[coordination/README.md](src/coordination/README.md)。

## 主要依赖

- ROS 2 Jazzy、`colcon`、MAVROS 和 PX4 SITL。
- Cosys-AirSim UE 仿真环境。
- Nav2/DWB、LiDAR 点云转换和 TF 相关 ROS 2 包。
- 项目根目录的 Python 3.12 虚拟环境；其中需要 `cosysairsim`、`numpy`、`pynput` 等依赖。
- `mpc_landing_demo` 和 `coordination_rtl_demo` 还需要 coni-mpc/ACADO；RTL demo 还需要 YOLOE 及其模型依赖。

## 构建

先启动 PX4 和 UE。可以在项目根目录运行：

```bash
cd ~/Air-UE-project
source .venv/bin/activate
./run_engine.sh
```

`run_engine.sh` 会清理旧的 PX4 SITL，启动 `none_iris`，等待 TCP 4560，然后打开 Blocks UE 工程。UE 打开后需要进入关卡并点击 **Play**。脚本内的 PX4、Unreal Engine 和工程路径按当前机器配置，换机器前需要检查。

在另一个终端构建 ROS 2 工作区：

```bash
cd ~/Air-UE-project
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd src/ros2
colcon build --symlink-install
source install/setup.bash
```

构建和运行 ROS 2 Python 节点前必须激活 `.venv`，否则节点可能找不到 `cosysairsim`。

## Wrapper 运行

车辆名称和命名空间约定如下：

| 车辆 | AirSim 名称 | 双实例 ROS 2 命名空间 |
|---|---|---|
| AGV | `AGV_1` | `/ugv` |
| UAV | `UAV_1` | `/uav` |

单实例 wrapper 的话题以 `/airsim_node/...` 开头；双实例 wrapper 使用 `/ugv/airsim_node/...` 和 `/uav/airsim_node/...`，避免话题冲突。

```bash
cd ~/Air-UE-project/src/ros2
source /opt/ros/jazzy/setup.bash
source ../../.venv/bin/activate
source install/setup.bash

# 单实例 wrapper
./start_wrapper.sh

# 双实例 wrapper
./start_agv_wrapper.sh
./start_drone_wrapper.sh

# 检查主要话题
./check_agv_topics.sh
./check_drone_topics.sh
```

## 键盘控制

这些脚本直接通过 Cosys-AirSim Python API 控制车辆，不需要 ROS 2 wrapper。

```bash
source ~/Air-UE-project/.venv/bin/activate

# 无人机：先使用 Multirotor settings 并让 UE 进入 Play
python3 ~/Air-UE-project/src/test/drone/keyboard_drone.py

# AGV：先使用 SkidVehicle settings 并让 UE 进入 Play
python3 ~/Air-UE-project/src/test/agv/keyboard_agv.py
```

`keyboard_drone.py`：`T/L` 起飞/降落，`M/N` 解锁/锁桨，`H` 悬停，`R` 重置，`Esc` 退出；`W/S/A/D` 平移，`Space/Shift` 升降，`Q/E` 偏航。

`keyboard_agv.py`：`W/S` 前进/后退，`A/D` 转向，`Q/E` 原地旋转，`Space` 手刹，`R` 重置，`Esc` 退出。AGV 停止时要使用刹车或手刹。

## 协同 Demo

以下命令均在已构建并 source 工作区的终端执行；三个 demo 不应同时控制同一架 UAV。

### 基础协同和避障

```bash
ros2 launch coordination coordination_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

启动 UAV wrapper、运动学 AGV、UAV 状态机、Nav2/DWB、点云转 LaserScan 和基础协调节点，执行 AGV 先行、UAV 起飞、导航、避障和降落流程。

### MPC 降落

```bash
ros2 launch coordination mpc_landing_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

在 AGV 追踪流程上加入 coni-mpc、姿态桥接和 MPC 降落协调器，使 UAV 接近并悬停后由 MPC 接管降落。

### 返航和视觉降落

```bash
ros2 launch coordination coordination_rtl_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

启动 AGV 巡航、UAV 状态机、Nav2、YOLOE、MPC 和 UAV/AGV follow 节点，执行去程、返航、视觉接近和降落流程。

`object` 必须是 UE World Outliner 中实际存在的 AGV actor 名称；默认`BP_HuskyVisual_C_1`。

## 主要话题

### AirSim wrapper

双实例模式下，AGV 使用 `/ugv/airsim_node/...`，UAV 使用 `/uav/airsim_node/...`。主要话题如下：

```text
# AGV
/ugv/airsim_node/AGV_1/car_state
/ugv/airsim_node/AGV_1/odom_local
/ugv/airsim_node/AGV_1/environment
/ugv/airsim_node/AGV_1/global_gps
/ugv/airsim_node/AGV_1/gps/AGV_1_Gps
/ugv/airsim_node/AGV_1/imu/AGV_1_Imu
/ugv/airsim_node/AGV_1/magnetometer/AGV_1_Magnetometer
/ugv/airsim_node/AGV_1/altimeter/AGV_1_Barometer
/ugv/airsim_node/AGV_1/distance/AGV_1_FrontDistance
/ugv/airsim_node/AGV_1/lidar/points/AGV_1_Lidar1
/ugv/airsim_node/AGV_1/lidar/labels/AGV_1_Lidar1
/ugv/airsim_node/AGV_1/front_center_scene/image
/ugv/airsim_node/AGV_1/front_center_scene/camera_info
/ugv/airsim_node/AGV_1/front_center_depth_planar/image
/ugv/airsim_node/AGV_1/front_center_depth_planar/camera_info

# UAV
/uav/airsim_node/UAV_1/odom_local
/uav/airsim_node/UAV_1/environment
/uav/airsim_node/UAV_1/global_gps
/uav/airsim_node/UAV_1/gps/UAV_1_Gps
/uav/airsim_node/UAV_1/imu/UAV_1_Imu
/uav/airsim_node/UAV_1/magnetometer/UAV_1_Magnetometer
/uav/airsim_node/UAV_1/altimeter/UAV_1_Barometer
/uav/airsim_node/UAV_1/distance/UAV_1_DownDistance
/uav/airsim_node/UAV_1/lidar/points/UAV_1_Lidar1
/uav/airsim_node/UAV_1/lidar/labels/UAV_1_Lidar1
/uav/airsim_node/UAV_1/front_center_scene/image
/uav/airsim_node/UAV_1/front_center_scene/camera_info
/uav/airsim_node/UAV_1/front_center_depth_planar/image
/uav/airsim_node/UAV_1/front_center_depth_planar/camera_info
```

公共话题包括：

```text
/ugv/airsim_node/origin_geo_point
/ugv/airsim_node/instance_segmentation_labels
/ugv/airsim_node/object_transforms
/uav/airsim_node/origin_geo_point
/uav/airsim_node/instance_segmentation_labels
/uav/airsim_node/object_transforms
```

单实例模式只需去掉 `/ugv` 或 `/uav` 命名空间，使用 `/airsim_node/...`。

### AGV 控制和协同链

```text
/ugv/airsim_node/AGV_1/car_cmd             airsim_interfaces/msg/CarControls
/ugv/airsim_node/AGV_1/odom_local           nav_msgs/msg/Odometry

/uav/state                                  std_msgs/msg/String
/uav/control/go_to_target_cmd               airsim_interfaces/msg/VelCmd
/uav/control/avoid_obstacle_cmd             airsim_interfaces/msg/VelCmd
/uav/control/landing_cmd                    airsim_interfaces/msg/VelCmd
/uav/control/approach_cmd                   airsim_interfaces/msg/VelCmd
/uav/control/yaw_setpoint                   std_msgs/msg/Float32
/uav/dwb_cmd_vel                            geometry_msgs/msg/Twist
/uav/dwb_goal_pose                          geometry_msgs/msg/PoseStamped
/uav/scan                                   sensor_msgs/msg/LaserScan

/mavros/local_position/odom                 nav_msgs/msg/Odometry
/mavros/global_position/global              sensor_msgs/msg/NavSatFix
/mavros/state                               mavros_msgs/msg/State
/mavros/extended_state                      mavros_msgs/msg/ExtendedState
/mavros/setpoint_raw/attitude               mavros_msgs/msg/AttitudeTarget
/mavros/setpoint_raw/local                  mavros_msgs/msg/PositionTarget
```

### 伪 AGV和恢复/MPC话题

```text
/sim_ugv/airsim_node/UGV_1/car_cmd          airsim_interfaces/msg/CarControls
/sim_ugv/airsim_node/UGV_1/imu/UGV_1_Imu   sensor_msgs/msg/Imu
/sim_ugv/airsim_node/UGV_1/odom_local       nav_msgs/msg/Odometry

/uav/airsim_node/UAV_1/recovery_state       std_msgs/msg/String
/uav/airsim_node/UAV_1/recovery_cmd         std_msgs/msg/String
/uav/coni_mpc/imu                           sensor_msgs/msg/Imu
/uav/coni_mpc/car_odom                      nav_msgs/msg/Odometry
/uav/coni_mpc/quad_odom                     nav_msgs/msg/Odometry
/uav/coni_mpc/attitude_target               mavros_msgs/msg/AttitudeTarget
/uav/coni_mpc/bridge_enable                 std_msgs/msg/Bool
```

### YOLOE 话题

```text
/uav/yoloe/detections                       std_msgs/msg/String
/uav/yoloe/target_pose                      geometry_msgs/msg/PoseStamped
/uav/yoloe/detected                          std_msgs/msg/Bool
/uav/yoloe/annotated_image                  sensor_msgs/msg/Image
/uav/yoloe/annotated_image/compressed       sensor_msgs/msg/CompressedImage
```

## 重要限制

- Cosys-AirSim 一个 UE 实例只能运行一个 `SimMode`；真实 Multirotor 和 SkidVehicle 不能在同一实例中同时运行。
- 双实例模式需要分别启动 UAV/AGV UE 实例，并使用不同 RPC 端口；单实例协同中的 `/sim_ugv` 是运动学 UE actor，不产生真实接触力。
- `odom_local` 是局部坐标系，不能直接比较 UAV 和 AGV 的绝对高度。
- UAV 绑定期间，`uav_follow_agv_node` 会通过 `simSetKinematics` 覆盖 UAV 状态，普通速度指令不会生效。
- `coordination_demo` 当前不启用恢复绑定节点；需要绑定、视觉降落或 MPC 时使用对应的 RTL/MPC demo。
