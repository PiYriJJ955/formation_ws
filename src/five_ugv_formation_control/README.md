# five_ugv_formation_control

五车编队中部署在 UGV1 上的位移跟随控制器。UGV0 是领航车；本节点读取两车
的 UWB 位置、`odom_combined` 航向，以及 UGV0 的里程计实测速度，向
`/ugv1/cmd_vel` 发布速度。

## 控制律

UGV1 相对 UGV0 的偏移 `d_1` 定义在领航车车体系（x 向前、y 向左）：

```text
p_1^d = p_0 + R(theta_0) d_1
u_1 = dot(p_1^d) + k_p (p_1^d - p_1)
```

默认 `d_1=(-0.30,+0.50)m`。UGV0 的速度读取自 `/ugv0/odom`
（`nav_msgs/Odometry`）的 `twist.twist.linear.x/y` 和 `twist.twist.angular.z`。
线速度位于 UGV0 车体系，使用已对齐的领航车航向转换到 UWB 坐标系：

```text
v_0_uwb = R(yaw_0_uwb) * [vx_body, vy_body]
dot(p_1^d) = v_0_uwb + omega_0 * J * R(yaw_0_uwb) * d_1
```

`/ugv0/odom_combined` 是 `geometry_msgs/PoseWithCovarianceStamped`，提供航向。
平面控制向量再转换为差速底盘的 `linear.x` 和 `angular.z`。

## 位置与航向协调

控制器使用三种运动状态：

- `FOLLOWING`：距离目标 ≥ `heading_blend_distance` 时，优先朝向位置控制向量。
- `APPROACH`：进入接近范围后，用平滑权重沿最短角度路径将运动方向向
  `UGV0航向 + offset_yaw` 偏转，同时加入加权的 UGV0 实测角速度前馈。
- `HOLD`：UGV0 静止且误差连续 `hold_enter_duration` 小于等于
  `position_tolerance` 后保持位置，线速度为零，只同步航向。
  误差超过 `hold_exit_tolerance` 或 UGV0 开始运动时退出。

UGV0 静止要求里程计平移速度和角速度同时处于各自死区内（或精确为零）。
保持期间 `heading_tolerance` 内角速度为零，超出死区后按剩余角度比例修正，
角速度不超过 `hold_max_angular` 和全局 `max_angular` 中较小的值。
禁用或数据无效/超时时停车并清除保持计时，恢复后重新判断。

默认参数在 `config/formation.yaml`：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `heading_blend_distance` | 0.50 m | 开始渐进同步航向的距离 |
| `heading_blend_max` | 0.35 | 接近阶段最大航向权重，必须小于 0.5 |
| `position_tolerance` | 0.10 m | 保持进入半径 |
| `hold_exit_tolerance` | 0.18 m | 保持退出半径 |
| `hold_enter_duration` | 0.4 s | 连续到达且静止的确认时间 |
| `heading_tolerance` | 0.08727 rad（5°） | 同步航向死区 |
| `hold_max_angular` | 0.35 rad/s | 保持角速度上限 |

接近阶段的角度权重从 0 平滑增加到 `heading_blend_max`，保持时为 1。
差速车需要允许一定航向差来纠正侧向位置误差，因此接近阶段保留位置方向的主导权；
线速度使用控制向量在车头方向的非负投影，转向误差较大时先原地旋转。
合成速度低于 `velocity_deadband` 时线速度也为零，并直接同步航向，避免使用
很小速度向量的方向。距离参数需满足
`0 < position_tolerance < hold_exit_tolerance <= heading_blend_distance`。
这些默认值用于初次低速实车验证，可按位置抖动幅度调整两个保持半径。

## UWB 与 odom 航向对齐

启动前让 UGV0 和 UGV1 都朝向 UWB +X。默认配置收到第一帧 odom 时计算：

```text
yaw_offset = initial_uwb_yaw - yaw_odom
yaw_uwb = yaw_odom + yaw_offset
```

如已有标定偏角，将 `auto_align_yaw` 设为 `false`，并填写
`leader_yaw_offset`、`self_yaw_offset`（弧度）。

## 编译与运行

```bash
cd /mnt
catkin_make --pkg five_ugv_formation_control
source devel/setup.bash
roslaunch five_ugv_formation_control follower.launch
```

UGV0 底盘需已启动并在同一 ROS Master 下发布里程计；检查速度输入：

```bash
rostopic type /ugv0/odom
rostopic echo /ugv0/odom/twist/twist
```

若里程计话题不同，用 `leader_velocity_odom_topic:=/实际里程计话题` 启动；
该话题的 `twist` 应为 UGV0 车体系下的实测速度。

确认目标点和航向正确后使能：

```bash
rostopic pub -1 /five_ugv_formation/enable std_msgs/Bool "data: true"
```

停车：

```bash
rostopic pub -1 /five_ugv_formation/enable std_msgs/Bool "data: false"
```

任一 UWB `valid=false` 或输入超时，控制器会发布零速度。UGV0 速度里程计也参与
输入检查：尚未收到、速度包含非有限数值或超过 `data_timeout` 未收到有效消息时，
控制器以 `STALE_OR_MISSING_INPUT` 状态停车。调试话题位于：

```text
/ugv1/formation_controller/target_pose
/ugv1/formation_controller/tracking_error
/ugv1/formation_controller/leader_velocity
/ugv1/formation_controller/desired_velocity
/ugv1/formation_controller/heading_error
/ugv1/formation_controller/heading_sync_error
/ugv1/formation_controller/heading_blend
/ugv1/formation_controller/state
```

## 编队数据记录

`follower.launch` 默认同时启动 `/ugv1/formation_logger`。停止 launch 时，数据和
图像保存到：

```text
/home/wheeltec/wheeltec_robot/formation_logs/ugv1_formation_时间戳/
```

每次实验包含：

```text
formation.csv
parameters.txt
summary.txt
01_trajectories.png
02_positions.png
03_tracking_error.png
04_heading.png
05_velocities.png
06_uwb_quality.png
07_availability.png
```

CSV 记录两车 UWB 位置、原始 odom 航向、目标点、位置和控制航向误差、领航速度、
期望速度、两车速度指令、控制状态，以及两车 UWB 有效性和残差质量。
`leader_odom_vx/vy/omega` 记录 UGV0 车体系下的原始实测速度，
`leader_velocity_odom_stamp/age` 记录该消息时间戳和距上次接收的时间；
`leader_vx/vy/omega` 是控制器使用的 UWB 坐标系速度（已应用速度死区）。
`05_velocities.png` 同时绘制里程计实测速度、控制前馈和速度指令。
`heading_error` 是当前控制方向误差，`heading_sync_error` 是对齐后
`UGV0航向 + offset_yaw - UGV1航向` 的归一化角度（弧度），`heading_blend`
记录航向同步权重。`04_heading.png` 同时绘制两种角度误差；
`07_availability.png` 绘制输入有效性、控制状态和同步权重。
汇总统计包含 `FOLLOWING`、`APPROACH`、`HOLD` 三种使能状态的样本。
需要指定目录或关闭记录时：

```bash
roslaunch five_ugv_formation_control follower.launch \
  log_root:=/home/wheeltec/wheeltec_robot/formation_logs

roslaunch five_ugv_formation_control follower.launch enable_logger:=false
```

若本机底盘订阅全局 `/cmd_vel`，通过 launch 参数连接即可：

```bash
roslaunch five_ugv_formation_control follower.launch cmd_vel_topic:=/cmd_vel
```
