# agv_actor_sim

Pseudo AGV simulation backed by a Cosys-AirSim UE actor (e.g. `UGV_Husky`).
The actor has no PhysX/Chaos vehicle physics; this package drives it
kinematically via `simSetObjectPose` and publishes IMU + Odometry derived
from successive pose samples.

## Topics

| Direction | Topic | Type |
|---|---|---|
| sub | `<prefix>/car_cmd` | `airsim_interfaces/msg/CarControls` |
| pub | `<prefix>/imu/<imu_name>` | `sensor_msgs/msg/Imu` |
| pub | `<prefix>/odom_local` | `nav_msgs/msg/Odometry` |

Default `<prefix>` = `/sim_ugv/airsim_node/UGV_1`, so default topics are:

```
/sim_ugv/airsim_node/UGV_1/car_cmd
/sim_ugv/airsim_node/UGV_1/imu/UGV_1_Imu
/sim_ugv/airsim_node/UGV_1/odom_local
```

`car_cmd` uses the same field semantics as the Cosys-AirSim wrapper (throttle/
steering/brake in [-1, 1] / [0, 1]).

## Nodes

| Executable | Role |
|---|---|
| `agv_actor_node` | Subscribes `car_cmd`, integrates motion, calls `simSetObjectPose` |
| `agv_imu_odom_node` | Samples actor pose at <=30 Hz and publishes IMU + Odometry |
| `agv_test_drive_node` | One-shot closed-loop test: forward, brake, backward, brake, spin |

Common parameters (all settable via `--ros-args -p name:=value`):

| Param | Default | Notes |
|---|---|---|
| `object` | `UGV_Husky` | UE actor name (World Outliner) |
| `topic_prefix` | `/sim_ugv/airsim_node/UGV_1` | Topic namespace, mirrors wrapper layout |
| `host_ip` | `127.0.0.1` | AirSim RPC host (launch file already fixes this for the single-instance setup) |
| `host_port` | `41451` | AirSim RPC port (launch file already fixes this for the single-instance setup) |
| `max_speed` (actor_node) | `2.0` | m/s when `throttle = 1.0` |
| `max_yaw_rate` (actor_node) | `90.0` | deg/s when `steering = 1.0` |
| `rate` (imu_odom_node) | `30.0` | Hz, capped at 30 |

## Build

```bash
cd ~/Air-UE-project/src/ros2
colcon build --symlink-install --packages-select agv_actor_sim
source install/setup.bash
```

## Run

UE must be in Play with the actor (default `UGV_Husky`) present.

Start both background nodes:

```bash
ros2 launch agv_actor_sim agv_actor_sim.launch.py \
  object:=UGV_Husky \
  topic_prefix:=/sim_ugv/airsim_node/UGV_1
```

Override actor name (e.g. when UE auto-named the instance):

```bash
ros2 launch agv_actor_sim agv_actor_sim.launch.py object:=BP_HuskyVisual_C_1
```

Send a single car command from the CLI:

```bash
ros2 topic pub --once /sim_ugv/airsim_node/UGV_1/car_cmd \
  airsim_interfaces/msg/CarControls \
  '{throttle: 0.6, steering: 0.0, brake: 0.0, handbrake: false, manual: false, manual_gear: 0, gear_immediate: true}'
```

Run the closed-loop test (forward 3 m -> backward -> spin 90 deg):

```bash
ros2 run agv_actor_sim agv_test_drive_node --ros-args \
  -p prefix:=/sim_ugv/airsim_node/UGV_1
# or pass CLI flags:
ros2 run agv_actor_sim agv_test_drive_node \
  --prefix /sim_ugv/airsim_node/UGV_1 --distance 3.0 --spin 90
```

Inspect the published streams (IMU + Odom use best-effort QoS):

```bash
ros2 topic list | grep sim_ugv
ros2 topic hz --use-wall-time /sim_ugv/airsim_node/UGV_1/imu/UGV_1_Imu
ros2 topic echo --qos-reliability best_effort /sim_ugv/airsim_node/UGV_1/odom_local
```
