# ROS 2 工作区

节点和三个协同 demo 详见 [`coordination/README.md`](src/coordination/README.md)。

## 主要依赖

- Cosys-AirSim UE 仿真环境和 Blocks 工程。
- ROS 2 Jazzy、`colcon`、MAVROS，以及 PX4 SITL。

## 构建

启动前请将项目根目录的 `settings.json` 放置于 `~/Documents/AirSim/settings.json`，然后在项目根目录启动 PX4 SITL 和 Unreal Engine：

```bash
cd ~/Air-UE-project
source .venv/bin/activate
./run_engine.sh
```

另开终端构建并加载 ROS 2 工作区：

```bash
cd ~/Air-UE-project/src/ros2
source ../../.venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

构建和运行 ROS 2 Python 节点前必须激活 `.venv`。

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

## 协同 Demo

以下命令均在已构建并加载工作区的终端执行，三个 demo 不能同时使用。

### 基础协同和避障

```bash
ros2 launch coordination coordination_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

执行无人机起飞、导航、避障和降落流程。

### MPC 降落

```bash
ros2 launch coordination mpc_landing_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

使用 CoNi-MPC 跟踪无人车并完成动态降落。

### 返航和视觉降落

```bash
ros2 launch coordination coordination_rtl_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

执行去程、返航、YOLOE-26 视觉接近和 CoNi-MPC 动态降落流程。

`object` 必须是 UE World Outliner 中实际存在的 AGV actor 名称，默认为 `BP_HuskyVisual_C_1`。

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
