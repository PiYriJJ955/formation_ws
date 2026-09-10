# five_ugv_mpc_formation_control

适配当前五车系统的非线性 MPC 领航—跟随编队包。每辆跟随车独立求解，领航编号可选。
控制台第三页在“定位方式”后选择 `five_ugv_mpc_formation_control`；
原 `five_ugv_formation_control` 仍为默认项。选择会保存，先“停止本次启动”再切换，
重新启动后仍需手动使能跟随。两种算法使用相同节点名和启动锁，禁止同时控制同一辆车。

## 坐标系与定位

- 位置来自两车 `/ugvN/uwb/pose`，同时要求 `uwb/valid` 有效。
  `five_ugv` 定位通常为 `uwb_map`；`linktrack` 适配器通常为 `linktrack_map`。
  两车 frame_id 必须相同、非空，运行中更换地图要求重启控制器。
- 坐标单位米，地图 X/Y 不互换，航向以 +X 为零、逆时针为正（弧度）。
  偏移定义在领航车车体系：X 向前、Y 向左。
  目标为 `p_d = p_L + R(yaw_L) [offset_x, offset_y]`。
- 运行航向使用 `odom_combined`，两车静止至少 0.4 秒时，从有效定位 pose 的
  orientation 标定各自固定的地图—里程计航向差。复用现有控制器的接收时刻对齐、
  有界状态外推和数据过期检查，不以启动时任意朝向充当地图 +X。
- 鲁棒 UWB 定位仍要求按其 `initial_uwb_yaw` 正确初始化。单标签静止测距不能自行
  确定绝对航向。LinkTrack 模式需设备位置和四元数已处于同一配置地图，沿用当前适配器
  的参考点；MPC 不额外交换坐标轴或补偿标签安装偏移。两车应使用一致的定位配置。
- 速度来自各车 `odom.twist.twist` 的车体系实测值。
  领航预测采用恒定车体速度/角速度模型，预测时域内持续旋转编队偏移，
  偏移点速度为 `R(yaw_L) [vx - omega*offset_y, vy + omega*offset_x]`。

旧 `src/MPC` 的 `x_ref=x+dx*sin(yaw)-dy*cos(yaw)`、
`atan2(dx,dy)`、固定 Float32 话题和旧定位链不适用于当前约定。
保留该目录作为参考，以 `CATKIN_IGNORE` 排除默认构建（其中旧 MPCv2 源文件已缺失）。
新包参考其滚动预测、终端代价和受约束优化思路，重新实现当前系统的数据适配和优化问题。

## 求解与控制接口

默认 20 Hz 控制，12 个预测步。第一步使用实际控制周期（通常 0.05 s），后续步长
0.15 s，预测约 1.70 s。状态为 `[x,y,yaw]`，输入为 `[v,omega]`，
使用差速/mini_4wd 的中点离散运动模型，发布的 Twist.linear.y 始终为零。

目标函数包含全时域位置误差、周期航向误差、速度前馈误差、输入变化和末端误差。
运动中跟踪偏移点切线；静止时先捕获目标位置，进入 HOLD 后再对齐
`leader_yaw + offset_yaw`。共享位移控制律只提供优化初值和 HOLD 朝向控制，
普通跟随的最终命令来自 MPC 求解。

使用 [SciPy SLSQP](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html)，
提供解析目标梯度和约束雅可比；上一周期解作为候选初值，每次只执行第一个控制量。
约束涵盖所有预测步的前进线速度、双向角速度及线/角加速度。
默认最大线速度 0.15 m/s、角速度 0.8 rad/s，加速度 0.15 m/s²、0.8 rad/s²；
在线硬限速、HOLD 位置锁定和异常停车优先于平滑。

复用现有话题：
`/five_ugv_formation/enable`、
`/ugvN/formation_controller/{state,target_pose,tracking_error,set_max_linear,max_linear}`，
以及现有日志和领航键盘/路径控制。领航控制不因跟随算法选择改变。

求解失败、不满足约束、非有限结果、超过求解预算或数据过期均输出零速度并清除旧解，
不会沿用失败前的预测命令。额外状态有 `MPC_TIMEOUT`、`MPC_SOLVER_FAILED`、
`MPC_INFEASIBLE`、`FRAME_MISMATCH`、`FRAME_CHANGED_RESTART`。
诊断话题为 `~mpc_solve_seconds`、`~mpc_cost` 和 `~mpc_solver_status`，
日志增加算法名、求解时间、代价和求解状态。

求解预算默认 0.04 s，在目标函数/迭代回调和返回时检查。SciPy 本机求解调用不能被
此预算硬抢占；这不是硬实时执行保证。应根据车载 CPU 的实测耗时选择预测长度和运行频率，
不要仅靠放宽预算掩盖持续超时。本包没有障碍物或车间碰撞约束；
仅适用于现有可达、无障碍的编队任务，不能把跟踪优化当作避障规划。

## 安装和启动

车端支持现有 ROS Melodic / Python 2.7 与 Noetic / Python 3。
在车端更新工作空间后执行：

```bash
bash scripts/build.sh
source scripts/env.sh
roslaunch five_ugv_mpc_formation_control follower.launch \
  ugv_id:=2 leader_id:=1 offset_x:=-0.8 offset_y:=0.8 \
  max_linear:=0.15 auto_enable:=false
```

`build.sh` 自动检查当前 ROS Python 的 NumPy/SciPy；缺少时调用
`scripts/install_mpc_dependencies.sh`，通过 apt 安装对应 Python 版本的系统包。
已有依赖则不访问 apt。GUI 的 Git 同步先上传依赖脚本并复用已保存的登录密码完成 sudo，
密码仅经 SSH 标准输入传递，不放入命令、文件或日志。独立构建/自动更新需要 root 或
免交互 sudo；无此权限会报告准确安装命令并停止构建，不会标记构建成功。
无新增 qpOASES、Eigen 或本地硬编码库路径依赖。

GUI 主机不需要 ROS 或 SciPy 才能选择算法；离线算法测试另装：
`python -m pip install -r src/five_ugv_mpc_formation_control/requirements-test.txt`。
调参修改 `config/mpc.yaml`，或通过 `mpc_config:=/path/to/mpc.yaml` 指定独立配置。
通用定位、偏移、限速与日志参数沿用 `follower.launch`。

## 验证

```bash
python3 test/test_mpc_formation.py
python3 test/test_fleet_workbench.py
source scripts/env.sh
python3 test/test_fleet_launch.py
```

离线测试覆盖解析梯度、旋转偏移、±π、约束、超时/失败停车、两种地图接口、
航向标定、在线限速、原位移算法回归，以及带 100 ms 输入延迟、8 mm 位置噪声、
约 0.2 s 底盘滞后的闭环仿真。仿真采用 1 s 求解测试预算以隔离测试机器调度影响，
同时记录实际求解耗时；它不证明车端 40 ms 预算可满足。

当前 Windows 已安装的离线测试环境：从仓库根目录运行
`.local/mpc-venv/Scripts/python.exe test/test_mpc_formation.py`。

2026-09-10 Windows / Python 3.10 离线结果：静止/车后到位 RMS 约 0.020/0.017 m，
S 弯 RMS 0.016 m、P95 0.027 m；左右弯与旋转 RMS 均低于 0.001 m。
这些是所述模拟场景结果，不是实车定位或跟踪精度。当前 Windows 环境尚未执行
Linux ROS 的 catkin 构建、真实 roslaunch 解析和实车测试。
