# formation_ws

五车 ROS 工作空间，车辆按 IP 升序编号为 ugv1–ugv5。
主机 `192.168.0.117` 发布代码，小车通过公开 HTTP 仓库拉取。

| 小车 | IP | 工作空间 | UWB 串口 | 车型 |
|---|---|---|---|---|
| ugv1 | 192.168.0.106 | /home/wheeltec/formation_ws | 待配置 | mini_4wd |
| ugv2 | 192.168.0.108 | /home/wheeltec/formation_ws | 待配置 | mini_4wd |
| ugv3 | 192.168.0.109 | /home/wheeltec/formation_ws | /dev/ttyCH343USB1 | mini_4wd |
| ugv4 | 192.168.0.110 | /home/wheeltec/formation_ws | 待配置 | mini_4wd |
| ugv5 | 192.168.0.114 | /home/wheeltec/formation_ws | 待配置 | mini_4wd |

五辆车的 ROS Master 统一为 `http://192.168.0.106:11311`，与 Git HTTP 服务相互独立。
原 `192.168.0.111`、`192.168.0.112` 两车已弃用，默认排除扫描。
源码完整保存在新工作空间，构建只依赖 `/opt/ros`，旧工作空间保留。

ugv2–ugv4 新增 IOT 的 UID、固定串口、测距/测角实测结果和编队采集命令见
[UWB IOT 设备与实测记录](docs/uwb-iot.md)，其中也记录了当前定位串口配置需要修正的位置。

## 小车编队控制台 GUI

主机桌面运行 Python 3.6+ 程序，五车统一使用 `mini_4wd`。
在仓库根目录安装 GUI 依赖（Ubuntu 18.04 默认 Python 3.6 使用 pip 21.x）：

```bash
sudo apt update
sudo apt install -y git python3-pip python3-tk gnome-terminal
python3 -m pip install --user --upgrade 'pip<22'
python3 -m pip install --user -r requirements.txt
```

然后检查桌面环境并启动：

```bash
python3 scripts/fleet_console.py --check-gui
python3 scripts/fleet_console.py
```

打开控制台会自动启动本机 Git HTTP 服务；窗口底部始终显示 Git server 状态，每 3 秒刷新。
程序自动读取本机网卡 IP，在底部状态栏显示 Git 地址；每 3 秒刷新，地址随网络变化更新。
点击状态可查看本机接收地址、推送凭据和已填入实际 IP 的推送命令；多网卡时列出各个可用地址。
新电脑首次运行时，“Git 仓库地址”默认使用检测到的本机地址；手动设置的仓库地址会记住。
服务未运行或启动失败时显示异常状态。关闭控制台后服务继续运行，供小车完成更新；主机重启后随下次打开控制台启动。
`--check-gui` 只检查桌面环境。

本机 Ubuntu 20.04 已验证 X11、Tk 8.6 与 Paramiko 可用；Ubuntu 18.04 尚未完整实测。
已使用 Python 3.6.9 通过完整三页界面、扫描与控制操作的离线测试。
`requirements.txt` 包含 Paramiko 和 PyYAML；Tk 与 GNOME Terminal 通过系统包安装。
主机控制界面不依赖本机 ROS。
车端需要已经编译的工作空间、ROS 环境和可读写的 `/dev/wheeltec_controller`。

1. 填写扫描范围和排除列表，支持 `192.168.0.1-254`、完整起止 IP、
   `192.168.0.0/24`、单个 IP，以逗号或空格分隔。默认扫描本地网段并排除主机
   `192.168.0.117`、两辆弃用车辆 `.111` / `.112` 与虚拟机 `192.168.0.136`，
   最多 4096 个地址，可设置 SSH 端口、超时、并发和循环间隔。
   `192.168.0.136` 固定屏蔽：不扫描、不登录，也不出现在车辆历史与 CSV 中。
2. 默认用户名 `wheeltec`；首次填写 SSH 密码（或设置 `FORMATION_SSH_PASSWORD`），后续自动记住。点击“扫描并连接”，
   程序检查 SSH 登录并保留成功连接。默认启用“SSH 连接后自动克隆 / 同步”：
   目标目录不存在时克隆 Git 仓库，已有工作空间则快进同步并按需编译，完成后配置
   `.bashrc`、更新服务和 linger；启用“连接后启动底盘”时，安装成功后启动底盘。
   默认仓库地址为 `http://192.168.0.117:8000/formation.git`，可修改并记住。
   循环检测重用在线连接，并重试断开的 IP。修改连接设置或排除列表后点击扫描应用。
3. 选择列表中的一辆车，等到底盘“就绪”，按住方向按钮控制它；松手、移出按钮、
   切换车辆或窗口失焦会停车。点击键盘控制按钮后可用 WASD 或方向键。
   空格、Esc 和“全部停车”会对所有连接发出零速度。线速度默认 0.10 m/s，
   角速度默认 0.40 rad/s，可调上限分别为 0.5 m/s、1.5 rad/s。
4. 名称使用 `ugvN` 格式：回车或离开输入框后自动 SSH 更新车端
   `~/.config/formation/robot.env` 的 `UGV_ID`，例如 `ugv2` 对应 `export UGV_ID=2`。
   更新会备份并读回校验；拒绝与已记录车辆重复的编号。离线失败时保留待应用编号，
   下次连接 / 同步时重试。新编号由新启动的 ROS 节点使用；已经运行的节点需要重启。
   备注独立保存，可填写任意说明。扫描结束和底盘状态变化时自动输出 CSV，
   也可点击“导出列表 CSV”指定路径。列表包含 IP、名称、备注、SSH/底盘状态、
   车型、电压和上次在线时间。

选中 IP 后，点击“同步选中车辆”可立即同步；Ctrl / Shift 可多选批量执行。
按钮也会为尚未连接的选中 IP 建立 SSH 连接。同步会先停止本界面启动的底盘，
成功后根据“连接后启动底盘”选项恢复启动；其他 ROS 任务使更新延期时显示“等待更新”。
本地修改、分叉或错误分支会保留并报告，不执行强制覆盖。
同步读取 HTTP 仓库的 `master` 分支；主机改动提交并 `git push` 后即可分发。
GUI 临时上传主机当前的安装脚本完成引导，车端受 Git 管理的文件按仓库版本更新。
新发现车辆分配并记住可用的 `ugvN` 编号，已有 `robot.env` 的编号和配置会保留。
新增车辆的 UWB 串口须在运行定位前按实物填写；GUI 底盘手动控制可独立使用。

“车端 ROS Master”列显示各车当前配置。选择一个或多个 IP，填写“编队 ROS Master IP”，
点击“更新选中 ROS Master”，程序经 SSH 将对应车辆
`~/.config/formation/robot.env` 中的 `ROS_MASTER_URI` 改成 `http://填写的IP:11311`，
备份原文件并原子替换，保留 `ROS_IP`、车型、编号和 UWB 等其他设置。
“读取选中配置”可重新读取车端值，单选时也会填入输入框。新值由新启动的 ROS 进程读取；
编队运行前在目标主机启动 `roscore` 并重开原有 ROS 节点。
GUI 的独立手动控制仍使用每辆车的回环 Master（11321）。

独立控制模式在每辆车启动自己的 ROS Master（本机回环地址、端口 11321），
通过 SSH 向这辆车的 `/cmd_vel` 发送速度。底盘同时回传里程计和电压；
只有收到里程计且速度订阅已连接才显示“就绪”。使用前结束现有底盘/编队 ROS
任务，释放串口；串口被占用时显示错误，可在释放后点击“启动已连接底盘”重试。
“断开全部 / 关闭底盘”和退出窗口会停车并结束本程序启动的 ROS 进程。

车端指令有效期为 0.4 秒，超时归零；持续丢失 SSH 心跳 3 秒后关闭底盘。
主机界面卡住时也会停止续发运动指令。程序自动临时上传兼容 Python 2/3 的
`scripts/fleet_bridge.py`，无需重新编译车端工作空间。
车端最近一次启动日志保存在 `~/.cache/formation-console/chassis.log`。

设置（含密码）、窗口位置、选中车辆和备注保存在
`~/.config/formation-console/settings.json`，文件权限为 `0600`；
SSH 主机密钥首次连接时记入同目录的 `known_hosts`，后续密钥变化会拒绝连接。
默认列表为 `~/.local/share/formation-console/fleet.csv`，CSV 不含密码。
可用 `--config /其他路径/settings.json` 启动独立配置。

本地检查（不向实车发送运动指令）：

```bash
python3 test/test_fleet_console.py
```

### 编队算法页

上方三个页签是“扫描与连接 / 编队算法 / 小车定位图”。第二页的车辆选择与第一页面
独立记住，Ctrl / Shift 多选参与车辆。列表同时显示 UGV 名称、IP、`UGV_ID`、SSH 和启动状态。
“编辑选中车 UWB 串口 / 编队偏移”支持识别并分别保存 LinkTrack 定位串口与 IOT 串口；
ugv3 的左右 IOT 按 UID 分别配置。第三页顶部的“启动 IOT 实时监视”按第二页选车启动
IOT launch，弹窗显示距离、水平角曲线，并记录原始 bag。详见 [IOT 端口与实时监视](docs/uwb-iot.md#控制台端口识别与实时图表)。
双击车辆行或点击“打开选中 SSH 终端”可直接登录，在终端输入其他命令。
两页的车辆列表均支持右键“打开 SSH 终端”和“删除选中车辆”；右键已选行保留多选，
右键其他行则选中该行。删除会移除本地记录、断开对应连接并关闭对应终端；涉及本次编队时先停止编队。
两页列表和保存的记录同步更新，重新扫描可再次发现车辆。

“所有小车串口权限（777）”按钮作用于列表中所有已登记车辆，无需多选。
程序自动 SSH 登录，使用第一页保存的密码完成 sudo，执行
`sudo chmod -R 777 /dev/ttyCH343USB*`，并在结果弹窗列出各车的设备和实际权限。
没有匹配设备、sudo 失败或连接失败都会分别显示。设备重新插拔或重启后可再次点击。

明天首次使用可按下面的顺序操作：

1. 在第一页确认 SSH、仓库和 Master IP。若准备直接运行编队，可关闭“连接后启动底盘”，
   扫描并等待自动同步完成。在第二页选中领航车和跟随车，新配置的领航车默认 `ugv1`，
   已保存的领航车选择会保留。ROS Master 主机与领航车可以是不同车辆。
2. 新车先点击“立即同步 Git 仓库”：自动登录、克隆 / 快进更新并按需编译，列表显示进度，
   结束后弹窗逐车显示结果。同步的是 HTTP 仓库已发布的 `master`；主机尚未提交 / push 的
   工作区修改需要先按下方“发布和更新”发布。运行中的 ROS 任务会阻止安装更新，需先停止。
3. 用“编辑选中车 UWB 串口 / 编队偏移”配置新车的真实串口和跟随偏移。串口、偏移
   沿用车端配置；保存的自定义值在执行“配置检查”或“快捷总启动”时通过 SSH 写回 `robot.env`。
   在第三页检查基站坐标和标签高度。车辆启动时朝向 UWB 地图 +X，供现有算法初始化航向。
4. 点击“快捷总启动”。程序停止控制台原先的独立底盘，关闭自动启动和循环扫描，然后依次
   同步所选车辆编号 / Master / ROS_IP / mini_4wd → 检查或启动 Master → 启动各车底盘与 UWB
   定位 → 启动跟随控制器 → 连接实时监视。Master IP 对应主机也需能使用同一 SSH 设置登录，
   且已安装工作空间。各车等待真实里程计和有效 UWB 数据，最长 45 秒；失败保留已经打开的
   终端供检查，后续步骤停止，结果弹窗说明失败的车辆。
5. 总启动完成时跟随尚未使能。先在第三页确认位置，在第二页“5 · 监视与使能”点击
   “使能编队跟随”。跟随车会朝各自目标行驶。定位无效、过期或线速度上限尚未确认时按钮拒绝使能，控制器自身
   也会检查输入是否及时。

下方五个步骤页签依次是“配置检查 / Master / 底盘与定位 / 跟随算法 / 监视与使能”，可以单独执行。
首次使用或修改车端配置后，先运行“检查并同步配置”；该步骤完成配置同步和启动文件上传。
单独启动 Master、底盘、跟随或监视时直接使用已有配置，保留对应的 ROS 就绪检查。
单独连接监视只登录 Master，编队偏移使用界面最近一次读取的车端值。
GNOME Terminal 按任务分窗口：各车底盘与定位合并在一个窗口，各车跟随控制器合并在另一个窗口；
ROS Master 和手动 SSH 各自使用独立的窗口。同类任务以标签页区分车辆，并复用对应窗口，
窗口关闭后自动新建。标签标题显示车辆、IP 和任务；Ctrl+C 停止当前命令后会进入
可输入命令的 SSH shell。启动日志区域可以折叠，终端窗口也可统一最小化。
已有 Master 会直接接入；重复串口占用或已有同名控制器会报错，便于先结束原来的启动任务。
更改参与车辆、Master、领航车或定位设置后，先停止本次启动，再用新设置启动。

“停止本次启动”停止本控制台启动的 ROS 终端和实时监视，手动打开的 SSH shell 保留。

### 小车定位图页

第三页点击“领航控制…”打开弹窗，选择“键盘控制”或“参考路径”。领航车沿用第二页设置，
修改领航车选择会停止当前控制，需要按新设置重新连接监视。打开弹窗和恢复上次设置都不会自动行驶。

键盘模式中，点击“点击启用 · WASD / 方向键”：W / ↑ 前进，S / ↓ 后退，A / ← 左转，D / → 右转。
支持组合按键：W+A / W+D 边前进边转弯，S+A / S+D 边后退边转弯，方向键可同样组合或混用。
松开一个键只取消对应的速度分量，全部松开停车；同轴反向键相互抵消，同向别名键不会叠加速度。
离开控制区、切换页签或关闭弹窗后需重新启用，停车前按住的键需要松开再按。
线速度和角速度与第一页共用并记忆，组合按键时全车线速度上限继续生效。

路径模式按以下步骤操作：

1. 连接实时监视，让领航车静止约 0.4 秒以建立航向参考。
2. 点击“地图选点”，弹窗暂时收起。在地图上点“清空路径”，再点“使用当前位置为起点”，
   然后按行驶顺序在空白处左键添加途经点；右键空白处或“撤销末点”可退回一步。紫色点和连线实时显示所选路径。
   **按住两点间的线段或中点圆环拖动，可将直线弯成曲线**；两端路径点保持不变，松开鼠标保存。
   已有曲线也可继续拖动调整；右键该线段或圆环可恢复直线。弯曲超出允许范围时，本次拖动不保存。
   至少两点、最多 50 点，相邻点间隔至少 0.15 m；超出基站范围或编队余量不足的点会提示原因并拒绝加入。
   也可离线选点，开始前需连接监视并确保起点距领航车不超过 0.5 m。
3. 点击“完成选点”返回弹窗，坐标框自动同步，也可手动修改：每行 `X, Y`（UWB 地图坐标，单位米），
   支持中英文逗号或空格分隔。手动修改坐标会将曲线恢复为直线；点击“预览路径”检查紫色参考线。
   巡航默认 0.10 m/s，预瞄默认 0.40 m（可调 0.3–0.5 m）。
4. 点击“开始”。如需整队跟随，先在第二页使能跟随，再开始领航路径；单车测试可保持跟随禁用。
5. “暂停”保留进度，“继续”由当前位置继续跟踪，“停止”结束本条路径。修改或重新选点前先暂停；
   选点不会自动行驶，点击“开始”才会使用编辑后的路径。关闭弹窗会暂停路径。

路径跟踪以 20 Hz 在 ROS Master 主机的监视进程中计算，GUI 只发送设置和控制操作。
所有领航速度经过同一个出口选择与限幅，切换模式时清零。拐角会平滑转向，实际轨迹可能在拐点内侧切弯；
起步方向偏差较大时先转向。接近终点减速，到终点 0.10 m 内停止。
定位无效、数据超过可调的过期阈值（默认 0.6 秒）、明显位置跳变、偏离当前路径超过 0.6 m 或底盘断开时暂停，恢复后需手动继续。
控制心跳过期 0.4 秒会停车并禁用跟随，重连后不会自动恢复路径。
Esc、键盘控制区内的空格键，以及“立即停车 / 禁用跟随”按钮可停领航车并禁用跟随。
普通路径暂停允许有效的跟随车继续收敛到其队形目标。
路径点、各段弯曲幅度、巡航速度、预瞄距离和模式选择会保存。曲线按不超过 0.05 m 的间隔采样，
地图预览和控制器使用相同的采样路径；最多 50 个途经点，曲线采样点不占途经点名额。
每次开始在 Master 主机工作空间的 `logs/leader_tracking/`
生成 `path.json` 和 `tracking.csv`，记录进度、横向误差、航向误差、实际下发速度、限速及暂停原因。
更新后重开 GUI 并重新连接实时监视，程序会自动上传 `leader_tracker.py` 和监视脚本。
旧监视进程不支持曲线路径时，点击“开始”会提示重新连接。
实时监视通过 Master 主机的 SSH 转发 ROS 数据，主机 GUI 无需安装 ROS。

“全车线速度上限”使用一个输入框，点击“应用到全部”后统一设置所有小车，并显示已生效数量。
范围为 **0–0.5 m/s**，默认 **0.15 m/s**。上限会保存，重开软件或新增车辆后均沿用。
领航键盘、路径跟踪及所有跟随控制器均受这一上限约束；在线下发后回读确认。
未参与本次监视的车辆在下次启动时应用。**0 限制平移，键盘转向仍可用**，路径跟踪会暂停；需要停车时使用停车按钮。
首次升级应先结束本次 ROS 任务、同步各车仓库，重开 GUI 后重新启动；旧控制器会显示待确认。
以后调整上限无需重启。建议全车上限设为 0.15 m/s，领航键盘请求速度设为 0.10–0.12 m/s，为追赶保留余量。
当前算法的数据来源、控制公式、状态切换与限速原理见 [编队算法说明](docs/formation-algorithm.md)。

图上显示基站、各车 UWB 位置、实际轨迹、目标轨迹和误差连线；下方显示各车控制器状态、
当前跟踪误差与最近最多 1000 个采样的误差 RMS。轨迹同样限制为 1000 点。
定位无效或超过“数据过期秒数”未更新会变灰，过期目标和误差不再显示为实时值。
该值默认为 0.6 秒，可在第三页修改；重连实时监视并重启跟随控制器后生效。
领航车箭头使用对齐到 UWB 地图的控制航向；跟随车箭头沿用连接监视时的朝向为 +X 参考。
重连监视时重新建立参考，领航车需静止等待对齐。

默认六个基站直接读取 `src/five_ugv_uwb_localization/config/final_localization.yaml`：
ID 0 / 4 / 1 位于下边，3 / 5 / 2 位于上边，场地 6.4 × 4.4 米，基站高度 1.35 米，标签高度 0.25 米。
“编辑基站坐标 / 定位参数”可增加、删除或修改 ID、X、Y、Z、车载标签高度和有效残差阈值；
残差阈值默认 0.6 米。
修改选中行后点击“修改选中行”，最后保存；至少保留四个 ID 唯一且不全部共线的基站。
自定义配置保存在控制台设置中，可恢复代码默认值。地图立即更新；
**正在运行的定位节点需要先停止再启动**，才会使用新基站坐标。

启动时控制台将完整定位配置（保留原来的解算参数）和当前启动入口上传到车端
`~/.cache/formation-console/<本次会话>/`，通过 `localization_config` 参数传给真正的定位节点。
坐标同时影响定位解算和地图显示；自定义场地的工作区边界按基站 XY 范围计算。

GUI 和 ROS 通信的离线检查命令：

```bash
python3 test/test_fleet_console.py
python3 test/test_fleet_workbench.py
python3 test/test_leader_tracker.py  # 直线、折线转弯、限速和暂停恢复模拟
python3 test/test_fleet_terminal.py  # 回环 SSH 测试，底盘 / 跟随两个窗口各打开两个测试标签页
source scripts/env.sh
python3 test/test_fleet_launch.py
python3 test/test_fleet_ros.py   # 仅启动回环地址 ROS Master 和模拟发布器，不启动底盘
```

## 发布和更新

GUI（`fleet_console.py`、`fleet_workbench.py`、`fleet_terminal.py`）在主机运行。
车端需要 ROS 包、启动文件、定位配置，以及运行中的遥测 / 控制脚本。
GUI 每次启动相应功能时会通过 SSH 上传当前主机的辅助脚本和基站启动入口，
因此不需要手工逐文件复制；正式版本仍应提交并推送到发布仓库，由小车更新工作空间。
**只有 commit 不会发布代码；commit 后还需 push。** 小车的更新定时器会自动 fetch，
在 ROS 停止且工作区干净时拉取、构建，也可通过 GUI 的 Git 同步按钮立即执行。
车端 `~/.config/formation/robot.env` 中的编号、Master 和串口配置独立保存，Git 更新会保留它们。

主机在本仓库提交并发布：

```bash
git add <修改的文件>
git commit -m "更新说明"
git push
```

主机的 `origin` 是本机 `.local/http/formation.git` bare 仓库。
Git Smart HTTP 服务使用 Git 自带的 `http-backend`，在同一地址提供 clone、pull 和 push：
`http://192.168.0.117:8000/formation.git`。拉取免密码，推送用户名为 `formation`，
专用密码在服务首次启动时生成，保存在主机 `.local/git-http-password`（权限 `0600`）。
点击控制台底部 Git server 状态可查看、复制连接信息。
`post-update` 继续调用 `git update-server-info`，保持旧客户端拉取兼容。

**从当前电脑推送到另一台电脑的本地 Git server**：接收电脑需有当前版本软件和 Git。
在接收电脑的仓库根目录安装服务，再打开控制台：

```bash
python3 scripts/git_http_server.py --install-service
python3 scripts/fleet_console.py
```

若接收电脑原有服务正在运行，安装后执行 `systemctl --user restart formation-git-http.service`。
点击**接收电脑**窗口底部状态，复制程序按本机网卡 IP 生成的推送命令，在**发送电脑**的仓库根目录执行。
例如当前这台电脑自动检测到 `192.168.0.117`，生成的命令为：

```bash
git remote add peer http://192.168.0.117:8000/formation.git
git -c credential.helper= push peer master:master
```

`origin` 用于当前主机发布，`peer` 用于指定的接收电脑。以后更换接收电脑，将它生成的
`git remote add peer 地址` 改成 `git remote set-url peer 地址`；向多台电脑发布时，可分别添加命名远端。
每台接收电脑会读取自己的 IP，并独立生成推送密码。
这些 Git 命令适用于 Windows / macOS / Linux。推送时输入**接收电脑**的用户名 `formation` 和专用密码。
`-c credential.helper=` 让这一次推送直接提示输入，兼容会拒绝明文 HTTP 的凭据管理器。
HTTP 连接未加密，适用于可信局域网。

推送发送已提交的 `master` 分支。若接收端有新的提交，先提交本地改动，再执行
`git pull --rebase peer master`，解决冲突后重新 push。接收端的提交进入其 bare 仓库；
接收电脑的开发目录也需自行 pull，小车按配置的仓库地址继续获取 `master`。
Git 服务支持协议 v0 / v2、gzip 和分块上传，单次 HTTP 请求上限为 512 MiB。

小车的 `formation-update.timer` 每分钟检查一次。检测到本机 ROS 正在运行时只 fetch；
ROS 停止后，工作区干净且位于 `master` 时执行快进更新与构建。
每轮输出保存在 `.local/update.log`（保留最近一轮）。更新或构建失败下轮重试；
本地修改、分叉和切换分支都需人工处理。
更新期间保持 ROS 停止。各车独立更新，开始实验前请核对五车提交号。
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

小车运行 ROS Melodic/Python 2，主机为 ROS Noetic/Python 3。
源码包含 `nlink_parser`、其串口与协议源码、`robot_pose_ekf`、底盘包和两个编队包。
系统仍需对应 ROS 发行版的常用消息包、serial、BFL、joint_state_publisher、
robot_state_publisher，以及 NumPy、Matplotlib。每辆车均需安装这些运行依赖。
主机已补装 `liborocos-bfl-dev`，可使用同一构建脚本编译完整工作空间。

```bash
cd /home/wheeltec/formation_ws
bash scripts/build.sh
source scripts/env.sh
```

每车设置在 `~/.config/formation/robot.env`，包括 `UGV_ID`、`ROS_IP`、`UWB_PORT`、
`CAR_MODE` 和跟随偏移；代码拉取不会覆盖这个文件。USB 重插后若设备编号变化，
需重新检查 `UWB_PORT`，不能与 `/dev/wheeltec_controller` 指向同一设备。

### CH343 串口驱动

ugv2（192.168.0.108）与 ugv4（192.168.0.110）使用 WCH 官方驱动，源码位于
`~/ch343ser_linux`。在 `driver/` 子目录针对当前内核编译 `ch343.ko`：

```bash
git clone https://github.com/WCHSoftGroup/ch343ser_linux.git ~/ch343ser_linux
cd ~/ch343ser_linux/driver
make
sudo modprobe -r cdc_acm   # 安装前先结束使用串口的程序，释放通用驱动
sudo make load
sudo make install
sudo usermod -aG dialout "$USER"
sudo reboot
# 重新 SSH 登录后检查
groups
ls -l /dev/ttyCH343USB*
modinfo ch343
```

两车配置 `/etc/modules-load.d/formation-ch343.conf` 加载 `ch343`，并在
`/etc/modprobe.d/formation-ch343.conf` 设置 `softdep cdc_acm pre: ch343`，
让 WCH 驱动先于通用 CDC ACM 驱动接管设备。`dialout` 组允许普通用户读写串口；
设备重建后无需重复添加组成员。内核版本改变后，需要重新编译并安装对应模块。

### 启动 ROS

在 ugv1（192.168.0.106）启动 ROS Master：

```bash
roscore
```

每台小车的新终端启动本车底盘和定位：

```bash
roslaunch five_ugv_uwb_localization ugv.launch
```

以下命令以 ugv1 为领航车，ugv2–ugv5 各用一个新终端启动跟随控制器：

```bash
roslaunch five_ugv_formation_control follower.launch leader_id:=1
```

编号默认读取 `UGV_ID`，也可显式传入 `ugv_id:=2`。命令行通过 `leader_id` 指定领航车，
GUI 使用界面保存的领航车选择。跟随偏移从各车的 `robot.env` 读取，按实际队形配置。
控制器启动时禁用。确认车辆初始朝向、目标点、定位有效性及五车版本后使能：

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
bash deploy/install-robot.sh ugv1   # 按上方 IP 表选择 ugv1–ugv5
```

安装脚本同步并构建工作空间、首次写入车端设置、备份并追加 `.bashrc` 环境入口，
启用用户级更新定时器和 linger。新终端自动选择本工作空间。
脚本支持 `ugvN` 编号；GUI 自动传入工作空间、仓库地址、本车 IP 和初始 Master IP。
已存在的车端配置会保留。

主机 HTTP 服务是用户级 `formation-git-http.service`，由控制台启动。
首次配置、升级服务或移动主机仓库目录后，在仓库根目录执行：

```bash
python3 scripts/git_http_server.py --install-service
# 已运行的服务升级后重启；首次安装则由控制台启动
systemctl --user restart formation-git-http.service
```

安装命令自动填入当前 Python 和仓库路径，支持不同用户名及带空格的目录，保持开机自启关闭。
首次安装若缺少 bare 仓库，会从当前已提交代码创建 `.local/http/formation.git`。
也可独立运行 `python3 scripts/git_http_server.py`；指定 `--repository /路径/仓库.git` 使用已有 bare 仓库。
单独运行时可用 `--port` / `--bind` 修改监听地址，控制台内置状态检查使用本机端口 8000。

运行状态也可在控制台底部查看。命令行检查与手动停止：

```bash
systemctl --user status formation-git-http.service
journalctl --user -u formation-git-http.service -n 30 --no-pager
systemctl --user stop formation-git-http.service
```

主机应保持 `192.168.0.117` 可达；地址变化时需更新小车 `origin` URL。
公开仓库不包含本地 `AGENTS.md`、凭据、服务数据、构建缓存和实验日志。

## 验证

以下检查不启动底盘：

```bash
python3 test/test_git_http_server.py  # 临时仓库、两个回环客户端；验证拉取、推送和认证
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


## 鲁棒 UWB EKF 与数据记录

定位节点直接融合每帧基站距离、本车 `odom` 的车体速度和 `imu` 的角速度，
估计 UWB 坐标系内的 `[x, y, yaw]`。启动文件限制 BLAS 为单线程，降低 Jetson 上小矩阵运算的调度开销。Q/R 的基准参数固定；按创新大小采用
Huber 降权和 3σ 拒绝，连续异常的链路隔离，连续正常后逐渐恢复权重。
每次更新至少需要 4 个基站，同时检查几何条件、残差、位置协方差和修正幅度。
默认每次测距修正限 6 cm，这限制的是预测后的校正量，并非车辆每帧的实际位移。

启动时车辆应静止并朝向 UWB +X，保持约 1 秒；其他初始朝向需修改
`final_localization.yaml` 中 `initial_uwb_yaw`，同时保持编队控制器朝向设置一致。
`tag_height` 是标签实际高度，`tag_offset_xy` 是标签相对底盘参考点的车体坐标偏移，单位米。
有非零偏移时，`uwb/pose` 的 XY 表示底盘参考点位置。

`uwb/valid` 只有在当前测距和运动数据都可靠、且连续更新通过后才为真。
`uwb/status` 给出 `INITIALIZING`、`TRACKING`、`RECOVERING`、
`INSUFFICIENT_ANCHORS`、`HIGH_RESIDUAL` 等状态。
`RESTART_REQUIRED` 表示运动数据缺失或设备时钟重启，缺失区间的转角无法可靠恢复；
停止本次启动，重新静止对齐朝向再启动。定位有效性与测量时效仍由编队控制器检查。
仅启动 UWB 时也必须已有同命名空间的底盘 `odom` 和 `imu`。

数据处理使用标签 `local_time` 毫秒时钟检测重复帧、乱序和相对积压，
只处理最新完整帧，不用上一帧距离补齐缺失基站。帧内每条距离的独立采样时刻和
固定串口传输延迟无法从当前 Nodeframe2 消息恢复；相关延迟不应被当作已经精确同步。
`uwb/pose_covariance` 给出含固定系统误差下限的协方差估计，它不是绝对精度保证。

每车 `logs/uwb/ugv*_test_*/` 从启动即创建记录，每秒 flush/fsync：

- `raw_linktrack.csv`：距离、FP/RX RSSI、接收时刻、标签本地/系统时间。
- `odom.csv`、`imu.csv`：运动测量及消息/接收时刻。
- `diagnostics.jsonl`：同一帧的状态、创新、归一化创新、权重、拒绝/隔离基站、协方差尺度和时效。
- `localization.csv`、`events.csv`：同帧定位质量及测距拒绝事件，包含无效帧。
- `parameters.json`：实际生效的基站、滤波和车端标定参数。

正常退出时先关闭数据文件，再生成图片。进程被强制结束时图片可能缺失；已经同步的
原始记录仍可用于分析，最近约一秒尚未同步的数据可能丢失。

### 固定测距偏差标定

目前偏差默认为零。每车使用至少 3 个相距 0.5 m 以上的独立测量标签参考位置，
在视距、静止条件下分别采集，每点每基站至少 30 帧。记录标签实际 `[x,y,z]`，
编写参考清单 `references.yaml`，例如：

```yaml
samples:
  - {csv: point_a/raw_linktrack.csv, tag: [1.0, 1.0, 0.25]}
  - {csv: point_b/raw_linktrack.csv, tag: [3.0, 1.0, 0.25]}
  - {csv: point_c/raw_linktrack.csv, tag: [3.0, 3.0, 0.25]}
```

示例坐标需要替换为实际测量值。生成并检查标定候选文件：

```bash
python src/five_ugv_uwb_localization/scripts/calibrate_uwb.py references.yaml \
  --config src/five_ugv_uwb_localization/config/final_localization.yaml \
  --output uwb_calibration.candidate.yaml
```

工具以参考点等权中位数估计固定偏差，并给出保守的固定距离标准差。
参考点之间偏差不一致时拒绝输出，避免用常数补偿位置相关的遮挡误差。
用独立于标定数据的参考点验证后，将该车的文件保存为
`~/.config/formation/uwb_calibration.yaml`，重启定位生效；普通 Git 更新和 GUI 基站配置不会覆盖它。
修改基站/标签安装后应重新验证标定。静态散布、跳变和有效率属于重复性/可用性指标，
绝对精度仍需独立测量真值；后续还要以直线、转弯、遮挡恢复数据验证动态误差和滞后。

离线验证：

```bash
source scripts/env.sh
python3 src/five_ugv_uwb_localization/test/test_range_ekf.py
python3 src/five_ugv_uwb_localization/test/test_realtime_localizer.py
```

修改前存档位于主机和各车 `.local/archives/pre-robust-ekf-20260908-124936/`。
车端原版本为 `c80f4e5`，保存了 Git bundle 与 robot.env；主机另存完整工作区归档和校验清单。
需要回退时先停止相关 ROS 启动和自动更新定时器，使用归档在单独目录恢复、构建并验证，
再切换运行工作空间，保留当前采集数据。
