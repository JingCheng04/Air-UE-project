# coordination

ROS 2 环境、依赖、构建方法和话题列表见 [`src/ros2/README.md`](../../README.md)。

## 运行准备

先在项目根目录启动 PX4 SITL 和 Unreal Engine：

```bash
cd ~/Air-UE-project
source .venv/bin/activate
./run_engine.sh
```

在UE中按照与AirSim相同的方式加载地图。另开终端构建并加载工作区：

```bash
cd ~/Air-UE-project
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd src/ros2
colcon build --symlink-install
source install/setup.bash
```

## 运行 Demo

基础导航和避障：

```bash
ros2 launch coordination coordination_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

追踪 AGV，基于YOLOE-26目标识别的视觉接近和CoNi-MPC 算法控制的降落：

```bash
ros2 launch coordination mpc_landing_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

去程、返航、基于YOLOE-26目标识别的视觉接近和CoNi-MPC 算法控制的降落：

```bash
ros2 launch coordination coordination_rtl_demo.launch.py \
  object:=BP_HuskyVisual_C_1
```

`object` 必须与 UE World Outliner 中的 AGV actor 名称完全一致。MPC demo 需要 coni-mpc/ACADO，RTL demo 还需要可用的 YOLOE 模型和相机输入。

## 注意事项

- 三个 demo 不得运行。
- `coordination_demo` 不启用 UAV/AGV 绑定；需要绑定、视觉接近或 MPC 降落时使用对应 demo。
- `/sim_ugv` 是通过 AirSim RPC 驱动的运动学 UE actor，不是真实 SkidVehicle，也不会产生接触力。
- follow 节点绑定 UAV 时会覆盖其运动学状态；自定义程序必须在起飞前发布 `recovery_cmd=release`，降落前切回 `auto`。
- `/uav/scan` 无数据时，应检查 UE 是否 Play、AirSim RPC、LiDAR 点云、静态 TF、点云过滤和 `pointcloud_to_laserscan`。
