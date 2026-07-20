# 避障算法详解

本文档说明 `coordination` + `navigation_bringup` 共同实现的 UAV 避障策略。当前系统不是单一算法，而是四层叠加：

1. **状态机层**：决定 GO_TO_TARGET / AVOID_OBSTACLE / LANDING
2. **DWB + local_costmap 层**：在 AVOID 状态下给出局部最优速度
3. **距离限速安全网**：根据最近障碍距离对速度再做一次物理约束
4. **反向人工势场（APF）**：主要用于解决 DWB 在凹角/死角的局部最优失效

最后一节给出当前代码中的所有关键参数值，便于复现。

---

## 1. 总体架构

```text
GPS target
   │
   ▼
ugv_then_uav_node
   ├─ GO_TO_TARGET: 直接朝目标飞行（cruise）
   ├─ AVOID_OBSTACLE: 关闭 cruise，交给 DWB / APF
   └─ LANDING: 到点后下降到 z≈0

AVOID_OBSTACLE 时：
   /uav/dwb_goal_pose  ─────►  dwb_goal_path_node  ─────►  controller_server(DWB)
           │                                                         │
           └──────────────────── APF + 安全网 + 高度环 ◄─────────────┘
```

最终唯一发到 AirSim 的速度话题是：

```text
/uav/airsim_node/UAV_1/vel_cmd_body_frame
```

---

## 2. 状态机层

实现文件：

- `coordination/coordination/ugv_then_uav_node.py`
- `uav_state_machine/uav_state_machine_node.py`

### 2.1 状态定义

- `GO_TO_TARGET`
- `AVOID_OBSTACLE`
- `LANDING`

### 2.2 GO_TO_TARGET → AVOID_OBSTACLE

状态机基于 360° 最近 LaserScan 距离：

$$
 d_{\min} = \min_i d_i
$$

其中 $d_i$ 是每束激光的有效距离。

进入 AVOID 的条件：

$$
 d_{\min} \le d_{\text{enter}}
$$

当前参数：

$$
 d_{\text{enter}} = 7.0\,\text{m}
$$

### 2.3 AVOID_OBSTACLE → GO_TO_TARGET

退出采用迟滞：

$$
 d_{\min} \ge d_{\text{clear}}\quad \text{持续}\quad T_{\text{hold}}
$$

当前参数：

$$
 d_{\text{clear}} = 9.0\,\text{m},\qquad T_{\text{hold}} = 0.4\,\text{s}
$$

此外，满足清场后并不会立刻恢复巡航，还会进入一段冷却期：

$$
 T_{\text{cooldown}} = 1.5\,\text{s}
$$

在这段时间里：
- `avoid_active` 仍保持为 True
- 不发 cruise
- 仍持续发布 DWB goal
- DWB / APF 继续把飞机推离最后一个障碍

这样做的原因是：如果刚躲开侧墙就立刻恢复朝目标的直线巡航，飞机很容易被拉回墙边，形成"改出 → 又撞回去"的循环。

---

## 3. GO_TO_TARGET 巡航层

实现文件：`coordination/coordination/ugv_then_uav_node.py`

目标的 GPS 偏差先转成局部 North-East 位移：

$$
 \Delta N = R_E \cdot \Delta\varphi,
$$

$$
 \Delta E = R_E \cdot \cos(\varphi) \cdot \Delta\lambda
$$

其中：
- $R_E = 6378137.0\,\text{m}$
- $\Delta\varphi$ 是纬度差的弧度
- $\Delta\lambda$ 是经度差的弧度

然后按当前偏航角 $\psi$ 投到机体系：

$$
 x_b = \Delta N\cos\psi + \Delta E\sin\psi
$$

$$
 y_b = -\Delta N\sin\psi + \Delta E\cos\psi
$$

巡航速度按距离做限幅：

$$
 v = \min(v_{\text{cruise}},\; 0.8\,d)
$$

其中：

$$
 d = \sqrt{\Delta N^2 + \Delta E^2}
$$

当前 demo launch 覆盖值：

$$
 v_{\text{cruise}} = 3.0\,\text{m/s}
$$

---

## 4. DWB 局部规划器

实现文件：

- `navigation_bringup/config/nav2_dwb_params.yaml`
- `navigation_bringup/navigation_bringup/dwb_goal_path_node.py`

### 4.1 rollout 公式

DWB 从当前速度采样多个候选 $(v_x, v_y, \omega)$，在预测时间窗 $T_{\text{sim}}$ 内积分：

$$
 \mathbf{p}(t) = \mathbf{p}_0 + \int_0^t \mathbf{v}(\tau)\,d\tau,
 \qquad t \in [0, T_{\text{sim}}]
$$

当前：

$$
 T_{\text{sim}} = 2.5\,\text{s}
$$

DWB 最大平面速度：

$$
 v_{xy,\max} = 2.5\,\text{m/s}
$$

所以单条轨迹最长预视距离约为：

$$
 L \approx v_{xy,\max} T_{\text{sim}} = 6.25\,\text{m}
$$

### 4.2 critic

当前启用 critic：

- `BaseObstacle`
- `PathDist`
- `GoalDist`
- `Oscillation`

总代价：

$$
 J = w_{\text{obs}}J_{\text{obs}} + w_{\text{path}}J_{\text{path}} + w_{\text{goal}}J_{\text{goal}} + J_{\text{osc}}
$$

其中最重要的是 `BaseObstacle`：

$$
 J_{\text{obs}} = \max_k c(\mathbf{p}_k)
$$

也就是说，只要轨迹上任一点撞进高代价区，整条轨迹就被高代价否决。

当前权重：

$$
 w_{\text{obs}} = 400,
 \qquad w_{\text{path}} = 0.001,
 \qquad w_{\text{goal}} = 0.001
$$

设计原则：
- AVOID 阶段 DWB 基本不再考虑朝目标更近，而是几乎只考虑远离障碍
- 目标推进交给上层状态机切回 GO_TO_TARGET 后的 cruise 去做

### 4.3 允许小幅后退

当前：

$$
 v_{x,\min} = -1.0\,\text{m/s}
$$

原因：
- 若完全禁止后退，DWB 在凹角里经常会出现 `No valid trajectories`
- 若后退范围太大（如 -2.5），则 DWB 会把"原地后退"当成最便宜的解
- `-1.0` 是折中：给 DWB 少量退路，但不让它无限后撤

---

## 5. local costmap 与 inflation

实现文件：`navigation_bringup/config/nav2_dwb_params.yaml`

当前 local costmap：

```yaml
width: 30
height: 30
resolution: 0.1
robot_radius: 1.0
inflation_radius: 4.0
cost_scaling_factor: 7.0
```

### 5.1 inflation 公式

采用指数衰减：

$$
 c(d) = (254-1)\,\exp\big(-k(d-r)\big)
$$

其中：
- $d$：离障碍 lethal cell 的距离
- $r = r_{\text{robot}} = 1.0\,\text{m}$
- $k = 7.0$

代价值近似：

| 距离 $d$ | 代价 $c(d)$ |
|---:|---:|
| 4.0 m | 0.2 |
| 3.0 m | 3.4 |
| 2.0 m | 31 |
| 1.5 m | 73 |
| 1.0 m | 253 |
| 0.5 m | 253 |

也就是说：
- 1m 内基本视为 lethal
- 1.5–2m 代价迅速抬升
- 3m 之外代价很小，不会让飞机在远离障碍时就被 costmap 束缚得不敢前进

这是当前设计中"近距离增加代价，而不是远距离到处都是高代价"的核心。

### 5.2 为什么不是更大的 inflation

我们实验过把 inflation 半径拉得更大，会让：
- 近墙确实更安全
- 但树缝、建筑夹缝直接被堵死
- DWB 找不到可行轨迹，只能悬停或后退

因此当前 4m 半径 + 7.0 scaling 是折中结果：
- 允许通过树缝
- 近距离贴墙时代价又足够陡

---

## 6. 距离限速安全网

实现位置：`ugv_then_uav_node._dwb_cmd_cb`

公式：

$$
 v_{\max}(d) = \sqrt{2 a_{\text{brake}}\,\max(d-s,\epsilon)}
$$

其中：
- $a_{\text{brake}} = 5.0\,\text{m/s}^2$
- $s = 1.0\,\text{m}$
- $\epsilon = 0.05\,\text{m}$

安全网只在：

$$
 d < 4.0\,\text{m}
$$

时启用。

也就是：
- 4m 外完全不管，让 DWB / APF 自己控制
- 4m 内按物理刹车距离限速
- stuck 状态下不启用，让 APF 有足够力脱困

这层不是为了替代 DWB，而是防止在最后 1–2m 里还保持太高速度撞上去。

---

## 7. 反向人工势场 APF

实现文件：`coordination/coordination/apf_escape.py`

### 7.1 设计目标

APF 不是主要避撞器，它主要用来：

1. 给 DWB 一个持续的反向偏置，避免在墙边悬停
2. 在 DWB 卡死（凹角、死角）时快速提供强推力
3. 在 `Failed to make progress` 反复 abort 时能接管 xy 速度

### 7.2 角度邻域加权

只取最近障碍束附近的角度窗口：

1. 找最近束方位 $\theta_c$
2. 取所有满足

$$
 |\theta_i - \theta_c| \le \frac{\Delta\theta}{2}
$$

的束，其中：

$$
 \Delta\theta = 2.618\,\text{rad} \approx 150^
$$

每束贡献力：

$$
 \mathbf{F}_i = -\frac{1}{d_i^2}(\cos\theta_i,\sin\theta_i)
$$

合力：

$$
 \mathbf{F}_{\text{base}} = \sum_i \mathbf{F}_i
$$

当前 APF 视距：

$$
 r_{\text{sight}} = 5.0\,\text{m}
$$

也就是：5m 之外 APF 沉默，只让 DWB 工作；5m 内 APF 开始给持续侧推。

### 7.3 stuck 检测

位置窗口长度：

$$
 T_w = 2.5\,\text{s}
$$

进入 stuck：

$$
 \Delta s < 0.5\,\text{m}
$$

退出 stuck：

$$
 \Delta s \ge 2.5\,\text{m}
$$

也就是说：
- 2.5 秒内移动不到 0.5m 就算卡死
- 一旦卡死，要移动 2.5m 以上才释放 stuck，防止 APF 刚推一下就被 cruise 拉回去

### 7.4 时变增益

总增益：

$$
 K(t) = K_{\text{base}} + K_{\text{stuck}}\rho^2(t)
$$

$$
 \rho(t)=\min\left(1,\frac{t-t_{\text{stuck}}}{T_{\text{sat}}}\right)
$$

当前值：

$$
 K_{\text{base}} = 2.2,
\quad K_{\text{stuck}} = 11.0,
\quad T_{\text{sat}} = 0.4\,\text{s}
$$

这意味着：
- AVOID 一开始就有明显推力
- stuck 后 0.4s 内迅速拉满
- 凹角里 DWB 一旦 silent，APF 几乎瞬间接管

### 7.5 输出限幅与低通

APF 输出限幅：

$$
 \|\mathbf{v}_{\text{APF}}\| \le 3.5\,\text{m/s}
$$

低通：

$$
 \mathbf{v}_k = \alpha\mathbf{v}_{k-1} + (1-\alpha)\mathbf{v}_{\text{raw},k}
$$

当前：

$$
 \alpha = 0.8
$$

## 7.6 DWB / APF 叠加与接管

在 `_dwb_cmd_cb` 中：

- 正常 AVOID：

$$
 \mathbf{v}_{\text{out}} = \mathbf{v}_{\text{DWB}} + \mathbf{v}_{\text{APF}}
$$

- stuck：

$$
 \mathbf{v}_{\text{out}} = \mathbf{v}_{\text{APF}}
$$

也就是 APF 在卡死时完全替代 DWB，解决 DWB silent 以后飞机原地悬停的问题。

---

## 8. 高度控制

AVOID 期间不允许高度漂移。

`_dwb_cmd_cb` 里始终发：

$$
 v_z = \text{clamp}\Big(-1.5(h_{\text{target}} - h),\,-1.0,\,1.0\Big)
$$

其中：
- $h = uav_z - hover_z$
- $h_{\text{target}} = uav\_height$

这让横向机动期间也锁住高度，不会因为 roll/pitch 倾斜导致爬升或下沉。

---

## 9. 激光点云预处理

### 9.1 自身点云过滤

文件：`navigation_bringup/lidar_self_filter_node.py`

采用圆柱过滤：

- `self_radius_xy = 0.60 m`
- `self_half_height = 0.25 m`

这是基于实测 LiDAR 自身反射点统计得到的近似机体包络。

### 9.2 LaserScan 转换

文件：`coordination/launch/coordination_demo.launch.py`

当前：

```yaml
min_height: -8.0
max_height: 8.0
range_min: 0.65
range_max: 120.0
```

意义：
- 允许足够多的高低点进 2D scan，便于把大楼、树冠、倾斜屋面都当成垂直墙看待
- `range_min = 0.65` 用于避免机体自身残余点重新进 scan

---

## 10. 为什么这套方案能处理超视距复杂障碍

### 10.1 大楼 / 大型复杂形状

远处：cruise 主导，飞机直接向目标飞行。

近处（7m）：切 AVOID，DWB 以障碍代价为主，APF 5m 内开始持续推开。DWB 决定"往哪边绕"，APF 负责避免在墙边悬停或 stuck。

### 10.2 凹角 / 死角

DWB 常见失败模式：
- `Failed to make progress`
- `No valid trajectories`
- 沿墙慢蹭

当前解决办法：
- DWB 先尝试
- stuck 触发后 APF 快速放大
- DWB silent 时 APF 完全接管
- `avoid_cooldown = 1.5s` 让飞机先彻底离墙再恢复 cruise

### 10.3 树缝 / 窄通道

当树离得不够近（>5m）时 APF 沉默，完全让 DWB 选通道；只有接近某一侧树的时候 APF 才在那一侧增强排斥力，帮助飞机走中间。

---

## 11. 当前参数列表（以代码为准）

### 11.1 coordination demo launch

```yaml
uav_height: 8.0
uav_ascend_speed: 3.0
cruise_speed: 3.0
obstacle_distance: 7.0
obstacle_clear_distance: 9.0
obstacle_clear_hold: 0.4
```

### 11.2 状态机 / coordination 节点

```text
target_tolerance: 1.0
landing_speed: 1.5
landing_z_tolerance: 0.2
avoid_cooldown: 1.5
safety_margin: 1.0
safety_gate_distance: 4.0
brake_decel: 5.0
```

### 11.3 DWB / local costmap

```yaml
min_vel_x: -1.0
max_vel_x: 2.5
min_vel_y: -2.5
max_vel_y: 2.5
max_speed_xy: 2.5
acc_lim_x: 3.0
acc_lim_y: 3.0
decel_lim_x: -5.0
decel_lim_y: -5.0
sim_time: 2.5
BaseObstacle.scale: 400.0
PathDist.scale: 0.001
GoalDist.scale: 0.001
robot_radius: 1.0
inflation_radius: 4.0
cost_scaling_factor: 7.0
required_movement_radius: 0.3
movement_time_allowance: 4.0
```

### 11.4 APF

```python
range_of_sight = 5.0
angular_window = 2.618   # 150 deg
free_distance = 4.0
window_seconds = 2.5
stuck_distance = 0.5
stuck_release_distance = 2.5
K_baseline = 2.2
K_stuck_extra = 11.0
saturation_time = 0.4
max_speed = 3.5
output_lpf_alpha = 0.8
```

### 11.5 点云过滤 / LaserScan

```yaml
self_radius_xy = 0.60
self_half_height = 0.25
min_height = -8.0
max_height = 8.0
range_min = 0.65
range_max = 120.0
```

---

## 12. 备注

这份文档描述的是当前代码状态。未来如果继续调 `BaseObstacle.scale`、`cost_scaling_factor`、`K_baseline` 或 `avoid_cooldown`，请同步更新本文件。
