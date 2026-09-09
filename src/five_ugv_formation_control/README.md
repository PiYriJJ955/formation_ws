# five_ugv_formation_control

可部署在 UGV1、UGV2 等跟随车上的位移跟随控制器。UGV0 默认是领航车；本节点读取两车
的 UWB 位姿、`odom_combined` 航向，以及两车的里程计实测速度，向
`/ugv<ugv_id>/cmd_vel` 发布速度。车辆编号默认读取车端 `UGV_ID`，也可通过
`ugv_id` 参数指定；领航车可通过 `leader_id` 参数指定。

## 控制律

UGV1 相对 UGV0 的偏移 `d_1` 定义在领航车车体系（x 向前、y 向左）：

```text
p_1^d = p_0 + R(theta_0) d_1
u_1 = dot(p_1^d) + k_p (p_1^d - p_1)
```

launch 默认 `d_1=(-0.8,+0.8)m`，车端 `UGV_OFFSET_X/Y` 可覆盖。UGV0 的速度读取自 `/ugv0/odom`
（`nav_msgs/Odometry`）的 `twist.twist.linear.x/y` 和 `twist.twist.angular.z`。
线速度位于 UGV0 车体系，使用已对齐的领航车航向转换到 UWB 坐标系：

```text
v_0_uwb = R(yaw_0_uwb) * [vx_body, vy_body]
dot(p_1^d) = v_0_uwb + omega_0 * J * R(yaw_0_uwb) * d_1
```

`/ugv0/odom_combined` 是 `geometry_msgs/PoseWithCovarianceStamped`，提供航向。
运动阶段以偏移目标轨迹的切线作为航向参考。记 `q=(vx-omega*d_y, vy+omega*d_x)`：

```text
v_d = |q|
theta_d = theta_0 + atan2(q_y, q_x)
omega_d = omega_0 + cross(q, dot(q)) / max(|q|^2, velocity_deadband^2)
e_body = R(-theta_1) * (p_1^d - p_1)
v = v_d*cos(theta_d-theta_1) + k_position*e_body.x
omega = omega_d + k_lateral*v_d*e_body.y + k_heading*sin(theta_d-theta_1)
```

`dot(q)` 从一阶滤波后的领航里程计速度变化计算，不对 UWB 位置做数值微分。
匀速恒曲率转弯时 `omega_d=omega_0`。目标速度低于 `velocity_deadband` 时，
用三次平滑权重过渡到 `u_1` 的位置捕获控制，静止目标也可从侧面或身后收敛。
运动过程中不要求所有车头平行；`offset_yaw` 用于到位后的最终朝向。

## 位置与航向协调

控制器使用三种运动状态：

- `FOLLOWING`：距离目标 ≥ `approach_distance`。
- `APPROACH`：距离目标 < `approach_distance`。与 FOLLOWING 使用相同的运动控制律，
  此距离只区分调试状态，不改变车头参考。
- `HOLD`：UGV0 静止且误差连续 `hold_enter_duration` 小于等于
  `position_tolerance` 后保持位置，线速度为零，只同步航向。
  误差超过 `hold_exit_tolerance` 或 UGV0 开始运动时退出。

UGV0 静止要求原始里程计平移速度和角速度连续 `stationary_duration` 低于各自阈值；
超过阈值的 1.5 倍即退出静止状态。这两个阈值只用于静止判定，运动前馈保留小角速度。
保持期间 `heading_tolerance` 内目标角速度为零，超出死区后按剩余角度比例修正，
角速度不超过 `hold_max_angular` 和全局 `max_angular` 中较小的值。
禁用或数据无效/超时时停车并清除保持计时，恢复后重新判断。

默认参数在 `config/formation.yaml`：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `k_position` / `k_lateral` / `k_heading` | 0.5 / 8.0 / 1.2 | 纵向位置、横向位置、切线航向反馈 |
| `approach_distance` | 0.50 m | FOLLOWING / APPROACH 显示边界 |
| `rotate_in_place_threshold` / `rotate_exit_threshold` | 0.95 / 0.55 rad | 原地对齐进入 / 退出阈值 |
| `max_acceleration` / `max_angular_acceleration` | 0.15 m/s² / 0.8 rad/s² | 正常运动的指令变化率上限 |
| `velocity_deadband` | 0.02 m/s | 低速位置捕获的平滑过渡尺度 |
| `position_tolerance` | 0.10 m | 保持进入半径 |
| `hold_exit_tolerance` | 0.18 m | 保持退出半径 |
| `hold_enter_duration` | 0.4 s | 连续到达且静止的确认时间 |
| `heading_tolerance` | 0.08727 rad（5°） | 同步航向死区 |
| `hold_max_angular` | 0.35 rad/s | 保持角速度上限 |
| `data_timeout` | 0.6 s | 两车位姿、速度及 valid 的接收超时阈值 |
| `prediction_limit` | 0.20 s | 位置、航向的最大外推时间 |
| `velocity_filter_tau` | 0.15 s | 里程计速度滤波时间常数 |
| `stationary_duration` | 0.4 s | 静止确认时间 |
| `leader_linear_deadband` / `leader_angular_deadband` | 0.01 m/s / 0.01 rad/s | 静止判定进入阈值，也用于本车初始标定 |

跟随车仅前进。航向误差从 0.55 增至 0.95 rad 时平滑降低目标线速度，进入原地对齐后，
误差降至 0.55 rad 才恢复前进。正常指令均经过加减速度限制；HOLD 位置锁定、
更低的硬限速、禁用和输入失效停车优先。距离参数需满足
`0 < position_tolerance < hold_exit_tolerance <= approach_distance`。
升级后 `heading_blend_distance/max` 不再参与控制，使用上表参数。
这些默认值用于初次低速实车验证，可按定位抖动调整保持半径。

## UWB 与 odom 航向对齐

先完成 UWB 定位初始化，再让两车保持静止至少 `stationary_duration`。默认使用有效 UWB pose
的 orientation，按本机接收时间插值 odom 航向，然后捕获固定偏移：

```text
yaw_offset = yaw_from_uwb_pose - yaw_odom_at_uwb_receive_time
yaw_uwb = yaw_odom + yaw_offset
```

如已有标定偏角，将 `auto_align_yaw` 设为 `false`，并填写
`leader_yaw_offset`、`self_yaw_offset`（弧度）。
自动标定在禁用期间也会进行；使能后尚未标定则以 `WAIT_ALIGNMENT` 停车等待。
此方法与领航路径控制使用同一 UWB 地图航向基准，不把跟随器启动朝向置零。
UWB 定位器自身的初始朝向要求仍需满足；单标签静止时不能凭距离独立测出绝对航向。

两车输入统一按本机接收时间判断新鲜度、记录航向历史和计算短时外推。
`data_timeout` 检测有效数据接收中断；非法数值和 UWB 无效仍触发停车。
使用里程计恒速模型从接收时刻向当前时刻外推，最多 `prediction_limit` 秒；
此方式不能识别或补偿消息到达前的链路延迟。
UWB 接收时刻无法与航向历史在外推范围内对应时，以 `INPUT_TIME_SKEW` 停车。

## 编译与运行

```bash
cd /home/wheeltec/formation_ws
bash scripts/build.sh
source scripts/env.sh
roslaunch five_ugv_formation_control follower.launch
```

UGV0 底盘需已启动并在同一 ROS Master 下发布里程计；检查速度输入：

```bash
rostopic type /ugv0/odom
rostopic echo /ugv0/odom/twist/twist
```

若里程计话题不同，用 `leader_velocity_odom_topic:=/实际里程计话题` 启动；
该话题的 `twist` 应为 UGV0 车体系下的实测速度。本车 `/ugvN/odom` 同样为必需输入，
可用 `self_velocity_odom_topic:=/实际本车里程计话题` 覆盖。

确认目标点和航向正确后使能：

```bash
rostopic pub -1 /five_ugv_formation/enable std_msgs/Bool "data: true"
```

停车：

```bash
rostopic pub -1 /five_ugv_formation/enable std_msgs/Bool "data: false"
```

任一 UWB `valid=false` 或输入超时，控制器会发布零速度。两车速度里程计也参与
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
/ugv1/formation_controller/reference_angular
/ugv1/formation_controller/state
```

## 编队数据记录

`follower.launch` 默认同时启动本车命名空间的 `formation_logger`。停止 launch 时，数据和
图像保存到：

```text
/home/wheeltec/formation_ws/logs/formation/ugv1_formation_时间戳/
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
`leader_vx/vy/omega` 是控制器使用的 UWB 坐标系速度（速度滤波，确认静止后置零）。
`reference_angular` 是偏移目标轨迹的切线角速度前馈，单位 rad/s；
`target_yaw` 是当前控制参考航向，HOLD 时为最终朝向。
`desired_vx/vy/speed` 保留 `dot(p_d)+k_position*位置误差` 的诊断向量，不代表最终底盘指令。
`05_velocities.png` 同时绘制里程计实测速度、控制前馈和速度指令。
`heading_error` 是当前控制方向误差，`heading_sync_error` 是对齐后
`UGV0航向 + offset_yaw - UGV1航向` 的归一化角度（弧度），`heading_blend`
运动时为 0、HOLD 时为 1。`04_heading.png` 同时绘制两种角度误差；
`07_availability.png` 绘制输入有效性、控制状态和同步权重。
汇总统计包含 `FOLLOWING`、`APPROACH`、`HOLD` 三种使能状态的样本。
需要指定目录或关闭记录时：

```bash
roslaunch five_ugv_formation_control follower.launch \
  log_root:=/home/wheeltec/formation_ws/logs/formation

roslaunch five_ugv_formation_control follower.launch enable_logger:=false
```

若本机底盘订阅全局 `/cmd_vel`，通过 launch 参数连接即可：

```bash
roslaunch five_ugv_formation_control follower.launch cmd_vel_topic:=/cmd_vel
```

## 离线验证

```bash
python src/five_ugv_formation_control/scripts/displacement_follower.py --self-test
python test/test_fleet_launch.py
```

自测不连接 ROS Master。覆盖接收超时保护、源时钟偏差、航向标定、限速、静止目标及身后目标收敛、
恒曲率零误差跟踪，以及左右圆弧、S 弯、领航原地旋转两侧共 8 个闭环场景。
动态场景加入 150 ms 输入延迟、约 1.2 cm 位置噪声、200 ms 底盘响应滞后，
输出稳定段 RMS、P95、转向换向次数和停车样本数。模拟结果不能替代实车验证。
首次试车建议保持上限 0.15 m/s、领航请求 0.06–0.08 m/s，先测圆弧，再测 S 弯与停车。
