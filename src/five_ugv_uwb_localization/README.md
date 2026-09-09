# five_ugv_uwb_localization

多车通用 UWB 定位功能包。它启动本车 LinkTrack、鲁棒二维定位器和可选离线记录器，
输出统一 `uwb_map` 坐标系中的本车位置。编号默认读取 `UGV_ID`，串口读取 `UWB_PORT`。
下面以 UGV1 的话题为例。

## 话题

```text
/ugv1/nlink_linktrack_nodeframe2
/ugv1/uwb/pose
/ugv1/uwb/valid
/ugv1/odom_combined
```

## 编译与启动

```bash
cd /home/wheeltec/formation_ws
bash scripts/build.sh
source scripts/env.sh
roslaunch five_ugv_uwb_localization ugv.launch
```

`ugv.launch` 按本车编号选择命名空间并启动底盘和 LinkTrack 节点，也可传入
`ugv_id:=2`、`port_name:=/dev/ttyCH343USB1`。只启动记录器时使用：

```bash
roslaunch five_ugv_uwb_localization offline_logger_only.launch
```

检查输出：

```bash
rostopic hz /ugv1/nlink_linktrack_nodeframe2
rostopic echo /ugv1/uwb/pose
rostopic echo /ugv1/uwb/valid
```

## 实时处理与队列

车端运行环境为 ROS Melodic/Python 2。测距订阅队列为 1，接收回调只保存最新消息及接收时间；
一个定时回调独占定位状态，默认以 20 Hz 处理最新帧。未处理的中间帧会被覆盖，
无新帧时不重复求解。输入仍可保持 50 Hz，离线记录器独立接收原始数据。

`config/final_localization.yaml` 中的实时参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `solve_rate` | 20.0 | 最新帧目标求解频率，Hz；过载时实际输出可以更低 |
| `max_measurement_age` | 0.15 | 从本节点接收起允许的最大结果年龄，秒 |
| `valid_max_residual_rms` | 0.6 | 定位结果允许的最大测距残差 RMS，米 |
| `position_lpf_reference_rate` | 50.0 | `position_lpf_alpha` 对应的参考频率，Hz |
| `path_publish_rate` | 2.0 | 轨迹最大发布频率，Hz；0 关闭轨迹收集和发布 |
| `path_max_points` | 1000 | 轨迹保留点数上限；完整历史使用离线日志 |

滤波系数按实际接收间隔换算：
`alpha(dt) = 1 - (1 - position_lpf_alpha) ** (dt * position_lpf_reference_rate)`。
这样降低求解频率或跳过帧时，滤波时间常数保持一致。

耗时和超时使用单调时钟；Python 2 通过标准库 ctypes 调用 Linux
`clock_gettime(CLOCK_MONOTONIC)`。求解前检查消息年龄，求解后再次检查消息及使用的缓存
测距是否过期；过期结果不更新位置或滤波状态。独立看门狗还会检查最后有效结果
的年龄，因此持续收到测距也不能掩盖定位输出停滞。看门狗检查周期为 0.05 秒。

`uwb/pose.header.stamp` 保存本定位节点接收该帧时的 ROS 时间。
LinkTrack 消息没有 ROS Header，所以这不是设备采样时间，也不能度量到达本节点
之前的串口、驱动或网络延迟。跨车消费者检查此时间戳时需要同步主机时钟。

新增诊断话题（均在 `/ugv1` 命名空间）：

- `uwb/measurement_age`：每次处理结束时，该帧距本节点接收的时间，秒。
- `uwb/processing_time`：该次处理总耗时，秒，包含求解和低频轨迹发布。
- `uwb/dropped_frame_count`：最新帧槽被覆盖的累计次数，不包含 ROS/TCP 层丢帧。

诊断按处理尝试发布，包括过期或无效的帧；输入停止后不再产生新的处理诊断。
50 Hz 输入、20 Hz 处理时覆盖计数正常增长，关注年龄是否保持稳定。处理结束时
的年龄包含位置发布后的轨迹处理时间，可能比该次位置发布时的年龄更大。

```bash
rostopic hz /ugv1/uwb/pose
rostopic echo /ugv1/uwb/measurement_age
rostopic echo /ugv1/uwb/processing_time
rostopic echo /ugv1/uwb/dropped_frame_count
```

连续运行时，年龄不应持续增长。输入停止后，`uwb/valid` 应在年龄/输入超时预算
加看门狗检查周期内变为 false。调参后需重启定位节点。

## 离线验证

使用真实 ROS 消息类型和合成测距验证帧覆盖、接收时间戳、慢求解期间接收与
看门狗独立运行、过期结果拒绝、缓存测距过期、轨迹限频限长及滤波时间常数。
以下测试在开发机的 ROS/Python 3 环境运行，使用标准库 `unittest.mock`，
无需启动 ROS master 或底盘：

```bash
source devel/setup.bash
python3 src/five_ugv_uwb_localization/test/test_realtime_localizer.py
```
