# formation_ws

三车 ROS 工作空间。主机 `192.168.0.117` 发布代码，小车通过公开 HTTP 仓库拉取。

| 小车 | IP | 工作空间 | UWB 串口 | 车型 |
|---|---|---|---|---|
| ugv0 | 192.168.0.111 | /home/wheeltec/formation_ws | /dev/ttyCH343USB2 | mini_4wd |
| ugv1 | 192.168.0.112 | /home/wheeltec/formation_ws | /dev/ttyCH343USB2 | mini_4wd |
| ugv2 | 192.168.0.109 | /home/wheeltec/formation_ws | /dev/ttyCH343USB1 | mini_4wd |

ROS Master 沿用 `http://192.168.0.112:11311`，与 Git HTTP 服务相互独立。
源码完整保存在新工作空间，构建只依赖 `/opt/ros`，旧工作空间保留。

## 发布和更新

主机在本仓库提交并发布：

```bash
git add <修改的文件>
git commit -m "更新说明"
git push
```

`origin` 是本机 `.local/http/formation.git` bare 仓库。`post-update` 调用
`git update-server-info`，Python 标准库 HTTP 服务只公开 `.local/http`。
匿名下载地址：`http://192.168.0.117:8000/formation.git`。HTTP 不接受 push；
发布通过主机本地文件路径完成。

小车的 `formation-update.timer` 每分钟检查一次。检测到本机 ROS 正在运行时只 fetch；
ROS 停止后，工作区干净且位于 `master` 时执行快进更新与构建。
每轮输出保存在 `.local/update.log`（保留最近一轮）。更新或构建失败下轮重试；
本地修改、分叉和切换分支都需人工处理。
更新期间保持 ROS 停止。各车独立更新，开始实验前请核对三车提交号。
不会自动启动底盘或控制器。

```bash
# 小车：立即更新并编译
cd /home/wheeltec/formation_ws
bash scripts/update.sh

# 检查版本、构建版本和更新服务
git rev-parse --short HEAD
cat .local/built-revision
systemctl --user status formation-update.timer
tail -n 50 .local/update.log
```

断网时已有代码可继续使用；恢复联网后重试。需要回退发布时，在主机
`git revert <提交>` 后 push，保留快进历史。维护期间可先
`systemctl --user stop formation-update.timer`，完成后再 start。

## 编译和运行

三车运行 ROS Melodic/Python 2，主机为 ROS Noetic/Python 3。
源码包含 `nlink_parser`、其串口与协议源码、`robot_pose_ekf`、底盘包和两个编队包。
系统仍需对应 ROS 发行版的常用消息包、serial、BFL、joint_state_publisher、
robot_state_publisher，以及 NumPy、Matplotlib。三车已安装这些运行依赖。
主机已补装 `liborocos-bfl-dev`，可使用同一构建脚本编译完整工作空间。

```bash
cd /home/wheeltec/formation_ws
bash scripts/build.sh
source scripts/env.sh
```

每车设置在 `~/.config/formation/robot.env`，包括 `UGV_ID`、`ROS_IP`、`UWB_PORT`、
`CAR_MODE` 和跟随偏移；代码拉取不会覆盖这个文件。USB 重插后若设备编号变化，
需重新检查 `UWB_PORT`，不能与 `/dev/wheeltec_controller` 指向同一设备。

在 ugv1 启动 ROS Master：

```bash
roscore
```

每台小车的新终端启动本车底盘和定位：

```bash
roslaunch five_ugv_uwb_localization ugv.launch
```

ugv1、ugv2 各用一个新终端启动跟随控制器：

```bash
roslaunch five_ugv_formation_control follower.launch
```

编号默认读取 `UGV_ID`，也可显式传入 `ugv_id:=2`。跟随目标默认是 ugv0，
可用 `leader_id` 调整。UGV1/UGV2 初始跟随偏移为 `(-0.8, +0.8)`、
`(-0.8, -0.8)` 米；请按实际队形在车端配置中调整。
控制器启动时禁用。确认车辆初始朝向、目标点、定位有效性及三车版本后使能：

```bash
rostopic pub -1 /five_ugv_formation/enable std_msgs/Bool 'data: true'
# 禁用
rostopic pub -1 /five_ugv_formation/enable std_msgs/Bool 'data: false'
```

定位与编队日志分别写入新工作空间的 `logs/uwb`、`logs/formation`。

## 新车配置与服务维护

```bash
git clone http://192.168.0.117:8000/formation.git /home/wheeltec/formation_ws
cd /home/wheeltec/formation_ws
bash deploy/install-robot.sh ugv0   # 按实际车辆选择 ugv0、ugv1、ugv2
```

安装脚本首次复制车端设置、构建、备份并追加 `.bashrc` 环境入口，启用用户级
更新定时器和 linger。新终端自动选择本工作空间。

主机 HTTP 服务是用户级 `formation-git-http.service`，已启用开机启动与 linger：

```bash
systemctl --user status formation-git-http.service
journalctl --user -u formation-git-http.service -n 30 --no-pager
```

主机应保持 `192.168.0.117` 可达；地址变化时需更新小车 `origin` URL。
公开仓库不包含本地 `AGENTS.md`、凭据、服务数据、构建缓存和实验日志。

## 验证

以下检查不启动底盘：

```bash
source scripts/env.sh
python test/test_fleet_launch.py
python src/five_ugv_formation_control/scripts/formation_logger.py --self-test
# 主机 Noetic/Python 3：
python3 src/five_ugv_formation_control/scripts/displacement_follower.py --self-test
python3 src/five_ugv_uwb_localization/test/test_realtime_localizer.py
python3 test/test_update.py
```

`robot_pose_ekf` 源码来自 ugv0 原工作空间（1.14.5）。`nlink_parser` 及内嵌依赖以源码
随仓库发布，保留上游许可证；导入前的 Git 元数据保存在主机 `.local/import-backups`。
