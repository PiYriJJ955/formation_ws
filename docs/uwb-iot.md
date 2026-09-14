# UWB IOT 设备与实测记录

2026-09-09 在 ugv2、ugv3、ugv4 上识别串口，并使用各车已编译的
`nlink_parser/iot` 同时采集约 60 秒。测试期间未启动底盘或编队控制。
设备型号为 **NIU_B01**。ugv3 左板 UID 为 `0x2A003200`，朝向位于其左侧的 ugv2；
右板 UID 为 `0x30006E00`，朝向位于其右侧的 ugv4。
本文初次记录中的 `ugv3` 对应右板，`ugv3_aux` 对应左板。

按安装方向完成的两分钟测试与结果图见 [NIU_B01 左右邻车链路实测](uwb-iot-left-right-test.md)。
六条有向链路的静止测试与 NIU_B01 手册说明见 [六链路静止测试](uwb-iot-six-links-static.md)。

## UID 与固定串口

| 设备 | IP | UID（十六进制） | UID（ROS 十进制） | 首次测试枚举串口 | USB 转串口序列号 | 固定路径 |
|---|---|---|---:|---|---|---|
| ugv2 | 192.168.0.108 | `0x15003B00` | 352336640 | `/dev/ttyCH343USB1` | `585A043472` | `/dev/uwb_iot` |
| ugv3 | 192.168.0.109 | `0x30006E00` | 805334528 | `/dev/ttyCH343USB1` | `5959047028` | `/dev/uwb_iot` |
| ugv3_aux | 192.168.0.109 | `0x2A003200` | 704655872 | `/dev/ttyCH343USB4` | `5959049274` | `/dev/uwb_iot_aux` |
| ugv4 | 192.168.0.110 | `0x30003000` | 805318656 | `/dev/ttyCH343USB1` | `5577010998` | `/dev/uwb_iot` |

UID 来自 IOT 协议帧，USB 序列号来自 `udevadm`，两者用途不同。
已在三辆车安装 `/etc/udev/rules.d/99-formation-uwb.rules`，重载规则并通过固定路径完成采集。
规则按 USB 序列号匹配，权限为 `0660`、组为 `dialout`，`wheeltec` 已属于该组。
规则源码为 [99-formation-uwb.rules](../deploy/99-formation-uwb.rules)。本次没有实测拔插或重启；
更换 USB 转串口模块后需要重新核对 UID 和 USB 序列号。

三车原有 LinkTrack 定位设备均在 `/dev/ttyCH343USB2`，也已建立 `/dev/uwb_linktrack`：

| 车辆 | LinkTrack USB 序列号 |
|---|---|
| ugv2 | `5959047708` |
| ugv3 | `5B7A090917` |
| ugv4 | `5B7A129523` |

编队定位的 `UWB_PORT` 应指向 `/dev/uwb_linktrack`。枚举路径可能在重插后改变，
请通过下面的设备识别功能核对；仅修改车端配置可能被控制台保存的设置覆盖。

## 控制台端口识别与实时图表

第二页选择一辆车，点击“编辑选中车 UWB 串口 / 编队偏移”，再点击“识别 UWB 设备”。
弹窗分别配置 `uwb_linktrack`、`uwb_iot`；ugv3 还会显示左板 `uwb_iot_aux`，并标注左右板 UID。
没有 LinkTrack 的车辆可将 `uwb_linktrack` 留空，仅保存和监视 `uwb_iot`；IOT 串口之间仍不能重复。
空闲串口按 921600 波特率读取并校验 NLink 协议帧，IOT 根据 UID 匹配左右板。
识别后优先填写固定路径，也可手动选择或输入端口。底盘、雷达别名对应的串口不参与探测；
已占用的串口会标注占用状态，设备类型如来自固定路径会明确显示“路径识别”。

“保存配置”会将 LinkTrack 和偏移保存在 `formation_config`，执行配置检查时写入车端；
IOT 端口单独保存在 `iot_ports`，下次启动 IOT 监视时使用。端口配置不会热切换运行中的节点。

第三页顶部点击“启动 IOT 实时监视”，按第二页选中的车辆启动采集并打开大窗口。
仅测试 ugv3 时，第二页只选 `192.168.0.109`。ugv3 同时启动两个 IOT，其他车辆启动一个。
窗口左侧为有向链路菜单，右侧为最近 60 秒的距离、水平角图表，菜单色块对应曲线颜色。
默认选中六条编队链路，收到其他链路时自动追加到菜单；未知对端显示十六进制 UID。
单击菜单项查看一条链路，Ctrl / Shift 多选叠加曲线；“全选”选择当前全部链路，“清空”清除选择。
对端缺测时图表留空；超出 NIU_B01 有效测角范围 ±50° 的点标红。
所有输出均保留，图表不增加平滑。横轴是主机接收时间，不代表设备时钟已同步。

点击“停止采集”会结束本次 IOT launch、保存 bag，并保留图表供查看。
关闭弹窗、断开全部或退出软件同样会停止采集；关闭弹窗后可再次点击入口开始新一轮。
连接失败、UID 不匹配或串口被占用时按车辆显示错误，其余车辆可继续采集。

IOT 使用每车回环 Master（11341），可独立于编队 Master 运行。
启动前核对车端 `UGV_ID`、实际 IOT 协议与来源 UID，禁止与底盘、LinkTrack 或另一块 IOT 共用串口。
临时上传 [fleet_iot_remote.py](../scripts/fleet_iot_remote.py) 和
[iot_monitor.launch](../src/nlink_parser/launch/iot_monitor.launch)，复用车端已编译的 `nlink_parser/iot`。

- 主机：`logs/uwb_iot/live_时间_编号/ugvN-IP.jsonl`，逐帧记录来源/对端 UID、
  设备时间、接收时间、距离、水平角和 RSSI，包含空对端帧。
- 车端：工作空间下同名 `logs/uwb_iot/live_时间_编号/`，包含完整原始 `iot.bag`、
  `iot.log`、`master.log` 和 ROS 日志。

2026-09-09 已在 ugv3 验证实际识别、双 IOT launch、图表与正常停止。
验证 bag 为 `logs/uwb_iot/live_20260909_135751_6ee711/ugv3/iot.bag`，
右板记录 173 帧、左板记录 172 帧；结束后两个 IOT 串口已释放。
此轮从 ugv3 接收对端广播，未测试其他车的控制台启动流程。
停止收尾复测为 `logs/uwb_iot/live_20260909_140358_shutdown_check/`，
右板 51 帧、左板 50 帧；主机 JSONL 与原始 bag 帧数一致，并收到保存完成确认。
端口、图表回归检查通过主机 Python 3.8 和兼容环境 Python 3.6，车端实际运行 Python 2。

```bash
python3 test/test_fleet_iot.py
source scripts/env.sh
python3 test/test_fleet_launch.py
```

## 采集结果

ROS bag 记录时间约为北京时间 **2026-09-09 10:57:47–10:58:49**。
每个设备的 `system_time` 相邻帧都递增 100，未观察到该计数重复、回退或间隔扩大。
设备标称输出约 10 Hz；各车主机时钟与设备时钟未经同步标定。

| 设备 | 帧数 | bag 时间跨度（秒） | 不含任何对端的帧数 |
|---|---:|---:|---:|
| ugv2 | 610 | 60.430 | 23 |
| ugv3 | 613 | 60.266 | 27 |
| ugv3_aux | 613 | 60.326 | 23 |
| ugv4 | 611 | 60.409 | 8 |

以下为均值 ± 总体标准差。出现率是“包含该对端的帧数 / 本机输出总帧数”，
不能直接解释为无线丢包率，也不能据此认定每条记录都是新的测量。

| 方向（帧来源 → 对端） | 样本数 | 出现率 | 距离（m） | 水平角（°） | 垂直角（°） |
|---|---:|---:|---:|---:|---:|
| ugv2 → ugv3 | 475 | 77.9% | 0.559 ± 0.014 | 74.89 ± 4.22 | −5.28 ± 3.23 |
| ugv3 → ugv2 | 473 | 77.2% | 0.559 ± 0.014 | −52.71 ± 5.33 | 57.45 ± 3.89 |
| ugv3 → ugv4 | 495 | 80.8% | 0.496 ± 0.019 | −32.15 ± 3.15 | 21.10 ± 2.57 |
| ugv4 → ugv3 | 496 | 81.2% | 0.498 ± 0.021 | −35.08 ± 3.24 | 8.82 ± 2.49 |
| ugv2 → ugv4 | 493 | 80.8% | 0.869 ± 0.016 | 7.16 ± 2.50 | 23.26 ± 2.74 |
| ugv4 → ugv2 | 485 | 79.4% | 0.868 ± 0.015 | 13.32 ± 2.44 | 19.56 ± 2.38 |
| ugv2 → ugv3_aux | 487 | 79.8% | 0.476 ± 0.012 | 68.53 ± 2.17 | −1.55 ± 3.12 |
| ugv3_aux → ugv2 | 485 | 79.1% | 0.477 ± 0.011 | 66.29 ± 3.17 | 0.90 ± 3.59 |
| ugv3 → ugv3_aux | 481 | 78.5% | 0.206 ± 0.021 | −54.39 ± 4.19 | 10.52 ± 5.40 |
| ugv3_aux → ugv3 | 473 | 77.2% | 0.206 ± 0.018 | −54.21 ± 5.17 | 3.41 ± 3.36 |
| ugv3_aux → ugv4 | 494 | 80.6% | 0.610 ± 0.068 | −37.74 ± 19.83 | 65.98 ± 6.12 |
| ugv4 → ugv3_aux | 479 | 78.4% | 0.607 ± 0.069 | −18.13 ± 2.51 | −36.63 ± 11.22 |

12 个有向链路均收到数据，本次记录的距离均为正且距离、角度均为有限数值。
主 IOT 的三组双向距离均值差约 0–2 mm，但这不代表绝对测距精度。
主设备间测距标准差约 1.4–2.1 cm；ugv3_aux ↔ ugv4 的距离标准差约 6.8–6.9 cm，
其中 ugv3_aux → ugv4 的水平角标准差达 19.83°，需要结合安装朝向、遮挡和实测真值进一步检查。
单条对端记录最大到达间隔约 0.595 秒，采集和后续算法应保留缺测，避免用上次值伪造连续数据。

角度是帧来源设备的局部测量，双向角度应分别保存。
设备安装方向、零角方向、正负方向与车体坐标变换尚未做实物标定，
不能直接把水平角当作车体/地图中的方位角；较大的垂直角也需结合安装核实。
本次验证覆盖设备识别、通信、解析和当前摆放下的输出统计，未验证运动编队中的误差或绝对精度。

## 编队时记录数据

每辆车沿用编队 ROS Master。先在车端终端启动对应 IOT，下面命令会读取该车 `UGV_ID`：

```bash
cd ~/formation_ws
source scripts/env.sh
roslaunch nlink_parser iot_base.launch \
  robot_name:=ugv${UGV_ID} uwb_name:=uwb_iot port_name:=/dev/uwb_iot
```

如需记录 ugv3 的第二个设备，在 ugv3 另一个终端运行：

```bash
cd ~/formation_ws
source scripts/env.sh
roslaunch nlink_parser iot_base.launch \
  robot_name:=ugv3 uwb_name:=uwb_iot_aux port_name:=/dev/uwb_iot_aux
```

在已接入同一 ROS Master 的主机或任意一辆车上记录全部 IOT 话题：

```bash
cd ~/formation_ws
source scripts/env.sh
mkdir -p logs/uwb_iot
rosbag record -o logs/uwb_iot/formation \
  /ugv2/uwb_iot/nlink_iot_frame0 \
  /ugv3/uwb_iot/nlink_iot_frame0 \
  /ugv3/uwb_iot_aux/nlink_iot_frame0 \
  /ugv4/uwb_iot/nlink_iot_frame0
```

按 Ctrl+C 结束记录。编队状态、里程计和定位仍由现有编队记录器采集；如需同 bag 分析，
可将实际的 `/ugvN/odom` 等话题追加到上述命令。
本次测试使用各车回环地址的临时 ROS Master（11341），测试结束已清理。
后续命令使用 `scripts/env.sh` 中的编队 Master，不需沿用临时 Master。

消息类型是 `nlink_parser/IotFrame0`：顶层 `uid` 为帧来源设备，`nodes[].uid` 为对端。
例如从 ugv2 话题中筛选 `msg.uid == 352336640` 且 `node.uid == 805334528`，得到 ugv2 → ugv3。
`dis` 单位为米；`aoa_angle_horizontal`、`aoa_angle_vertical` 单位为度；
`fp_rssi`、`rx_rssi` 按现有解析器转换为 dBm。
消息没有 ROS Header 或对端独立采样时刻，应同时保留接收时间与来源设备 `system_time`；
跨设备 `system_time` 不可直接作为同一时间轴，也不应把到达时间当作精确测量时间。

## 数据位置与复核

主机目录：`logs/uwb_iot/20260909_static_60s/`。
每辆车保留 `/home/wheeltec/formation_ws/logs/uwb_iot/20260909_static_60s/`。
主机每个 `ugvN/` 子目录包含：

- `iot.bag`：原始 ROS 消息，包含 UID、时间、角度、距离、RSSI 和用户数据。
- `frames.csv`：每帧一行，包含空节点帧，可复核输出频率和对端出现率。
- `measurements.csv`：每条有向链路一行，包含来源/对端 UID、接收时刻、帧序号、
  设备 `system_time`、距离、水平/垂直角、FP/RX RSSI、用户数据十六进制。
- `uwb_iot.log`、`rosbag.log`、`roscore.log`；ugv3 还包含 `uwb_iot_aux.log`。

主机根目录另有 `frame_summary.csv`、`pair_summary.csv`、`summary.json`；
`logs/uwb_iot/discovery.jsonl` 和 `inventory.json` 保存串口探测与映射证据。
日志目录按项目规则被 Git 忽略，原始数据已保留在本机和各车，未加入版本库。
