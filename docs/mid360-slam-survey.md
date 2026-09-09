# MID360 SLAM 选型调研

调研日期：2026-09-08。围绕现有小车的室内建图、定位与编队用途，核查官方规格、论文摘要、上游 README、配置及部分源码。本文的算力适配判断属于选型建议；本次尚未安装、编译或实测各 SLAM 算法。

**建议：现有 Jetson Nano 先验证 FAST-LIO2；需要完整回环、多次建图与全局优化时评估 Voxel-SLAM；升级计算平台后可重点考虑 GLIM；Point-LIO 和 FAST-LIVO2 分别作为高动态、视觉融合的研究对照。**

## 1. 实际平台与传感器

| 项目 | 已核实状态 |
|---|---|
| 测试小车 | `192.168.0.111` |
| 计算平台 | Jetson Nano，4 核 Cortex-A57，约 4 GB 内存，aarch64 |
| 系统 | Ubuntu 18.04.5，ROS Melodic，L4T R32.5.0 |
| 雷达 | MID360：`192.168.199.198` |
| 接收网口 | USB RTL8152，`eth1 = 192.168.199.10/24`，已持久配置 |
| 通信验证 | Ping 3/3 成功；抓到点云 `56300 → 56301`、IMU `56400 → 56401` 数据 |
| 现有控制 | 五车控制器以 20 Hz 工作，位置使用 UWB，航向使用 `odom_combined` |

Livox 官方规格列出：水平视场 360°，垂直视场 −7°～52°，点频 200,000 点/秒，典型帧率 10 Hz，内置 ICM40609 IMU，100BASE-TX 网口。标称探测距离为 40 m（10% 反射率）或 70 m（80% 反射率），均有测试条件。标称测距精度不能直接当作 SLAM 定位精度。[S1]

MID360 的非重复扫描适合直接点云配准类算法。选型应优先确认逐点时间、IMU 数据和扫描模式支持。Jetson Nano 的 GPU 不会自动加速 FAST-LIO2 等以 CPU 为主的实现；其他 ARM 平台的论文结果也不能直接换算为本车实时性能。

## 2. 代表方案比较

“已有 MID360 配置”与“能直接连接当前 driver2”是两个独立条件。下面分别列出。

| 方案 | 核心方法与优势 | 回环／长期定位能力 | MID360 与现有平台适配 | 建议 |
|---|---|---|---|---|
| **FAST-LIO2** | 迭代卡尔曼滤波，原始点到地图直接配准，ikd-Tree 增量地图；前端精简 | 主仓提供里程计与建图；回环、已有地图重定位需扩展 | 官方有 `mid360.yaml` 和 launch；支持 Melodic、ARM，但主仓仍依赖第一代 Livox 消息 | **现有 Nano 的首个基线** |
| **Point-LIO** | 按点的采样时刻进行状态估计；重点解决高速运动、振动和 IMU 饱和 | 主要是里程计与建图；完整后端需另行接入 | 当前默认分支文档仍使用第一代消息；官方测试 Ubuntu 20.04／Noetic，并明确建议避开 18.04 及以下 | 高动态研究对照，当前车端部署成本较高 |
| **Voxel-SLAM** | 自适应体素地图；包含初始化、里程计、局部优化、回环、全局优化 | 有跨 session 回环、全局 BA，并提供重定位实验 | 官方有 MID360 配置与数据示例；依赖 Noetic、PCL 1.10、GTSAM 4.0.3，仍声明第一代 Livox 驱动依赖 | **完整 SLAM 的优先比较对象**，先在兼容主机离线评估 |
| **GLIM** | 基于因子图的多帧／子地图配准，全局优化；有 CPU 与 GPU 实现 | 有全局一致性优化、地图编辑和多次建图合并；显式回环检测有扩展模块 | 官方明确列出 MID360；当前测试平台为 Ubuntu 22.04／24.04、Orin JetPack 6.1，ROS1 包已在关联包列表中划除 | 适合新工作站或 Orin；当前 Nano 环境迁移成本高 |
| **FAST-LIVO2** | 激光、IMU、相机通过顺序滤波更新融合；几何与图像信息互补 | 主要是融合里程计与建图；不能据此认定已有完整回环／全局重定位 | 主仓示例含 Avia 等，本次未发现专用 MID360 配置；需要核对消息接入，新增相机标定与同步 | 当激光退化确实影响任务时再加入视觉 |
| **LIO-SAM 原版** | IMU 预积分、特征匹配、因子图；易理解回环与绝对观测因子 | 有回环示例及 GPS 因子 | 原版文档要求机械式雷达的 ring／time 信息，并要求提供姿态的九轴 IMU；MID360 内置 IMU 不能直接满足其原版使用假设 | 可作为经典论文和后端参考；适配版需单独审查 |

依据：[S2]–[S7]。这些方案没有在本车、同一数据集上公平实测，不能给出可靠的精度排名或固定 CPU 占用排名。

## 3. 选型中最重要的区别

### FAST-LIO2：适合先建立可用基线

FAST-LIO2 的核心贡献是取消手工特征提取，以原始点直接匹配地图，并用 ikd-Tree 维护增量地图。官方声明支持 Melodic 和多种 ARM 平台，环境与本车更接近。[S2]

官方主仓确实已有 MID360 配置，其 `scan_line = 4` 是该实现使用的通道参数，不能解释成机械式四线雷达。配置中的 `det_range = 100` 属于算法参数，也不是雷达的官方有效量程。

**需要解决的实际接入问题：**本次查到 `CMakeLists.txt` 仍依赖 `livox_ros_driver`，`src/preprocess.h` 仍包含 `livox_ros_driver/CustomMsg.h`；MID360 的官方驱动则是 `livox_ros_driver2`。因此部署时应选择经过核查的 driver2 适配版本，或对消息依赖、订阅和预处理做必要适配，然后验证字段语义与时间单位。只修改话题名称不足以证明兼容。[S2a][S2b][S8]

主仓的 MID360 配置默认开启累计 PCD 保存，`interval = -1` 会累计到一个文件，配置注释明确提到长时间运行的内存风险。4 GB Nano 的首次在线测试应关闭无限累计保存和车端 RViz，使用有界录包／地图保存策略；地图范围、降采样与发布负载需要根据实测调整。[S2a]

### Point-LIO：高频能力要对应实际任务

Point-LIO 官方列出的特点包括 4–8 kHz 里程计输出，以及对 IMU 饱和、强振动和激烈运动的鲁棒性。这是论文／项目能力描述，不是本车达到的频率，也不等于点云帧率。[S3]

当前编队控制是 20 Hz，第一阶段更值得比较延迟、转弯漂移、退化恢复和资源占用。除非实验发现明显的高速运动问题，否则没有必要仅为了高输出频率增加当前系统的部署负担。其 Ubuntu 18.04 兼容性提示是本车选型的现实限制。

### Voxel-SLAM 与 GLIM：完整建图候选

Voxel-SLAM 将局部激光惯性 BA、回环和分层全局 BA 组成完整系统，官方提供 MID360 launch、跨 session 和重定位实验。它更适合回答“多次经过同一区域是否保持地图一致、重启后能否恢复定位”等问题。完整性也带来更多依赖和计算任务，应先在 Ubuntu 20.04／Noetic 环境用同一份数据与 FAST-LIO2 比较。[S4]

GLIM 提供 CPU 路径和 GPU 路径，并不是必须有 GPU 才能运行。其当前维护环境明显新于本车，官方还给出了 MID360 配置指南、全局优化、人工地图修正和多次地图合并能力。用于生产固定地图时值得评估；若要求开机后自动在固定地图定位，需要进一步核查相应定位模块、初值和失败恢复流程，不能只凭“能合并地图”认定完整具备。[S5]

### FAST-LIVO2：有相机条件时的扩展方向

FAST-LIVO2 将点云几何残差与图像光度残差加入同一估计框架，在部分激光几何退化场景中有互补价值。相机引入内参、激光／相机外参、曝光和时间同步要求；当前只接通 MID360 的条件尚不能直接复现完整 LIVO 系统。[S6]

其官方 README 还引用了资源受限平台版本：采用退化感知的视觉帧选择和更节省内存的地图结构。论文在 Hilti 数据上报告相对 FAST-LIVO2 的运行时间降低 33%、内存降低 47%，同时 RMSE 增加 3 cm。这些是特定实验的权衡结果，不能直接作为 Nano 上的预期收益；可作为后续研究线索。[S6a]

## 4. FAST-LIO2 的两类扩展

| 扩展 | 解决的问题 | 已核实限制 |
|---|---|---|
| [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM) | FAST-LIO2 前端加 Scan Context 回环和 GTSAM 位姿图优化 | README 的主要教程是 Ouster／MulRan；MID360 实时接入、回环阈值与算力仍需验证 |
| [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) | 高频里程计加低频已有地图匹配 | 需要初始位姿，可来自 RViz 或另一定位源；依赖 Python 2.7、旧版 Open3D，ARM 部署需核查 |

这里应分别验证“当前运行中闭环”“已有地图定位”“未知初始位置的全局重定位”“定位失败后的恢复”。保存 PCD 文件只完成了地图输出，不能替代这些能力。[S9][S10]

## 5. 本项目接入注意点

**数据格式。**优先使用 driver2 的 `msg_MID360.launch`／`xfer_format = 1`，保留 `timebase`、每点 `offset_time`、`line` 和 `tag`。driver2 的专用 PointCloud2 格式也带逐点 `timestamp`，因此不能笼统说 PointCloud2 没有时间；关键是算法是否正确解析实际字段。标准 XYZI 格式则缺少这类逐点时间信息。[S8]

**IMU 与外参。**先使用雷达内置 IMU，静止检查加速度模长、角速度、方向和时间戳。不同驱动／算法可能以 g 或 m/s² 表示加速度，不能统一盲目乘 9.81。GLIM 文档就明确要求按输入单位配置 `acc_scale`。雷达到内置 IMU 的外参，与整个传感器到小车 `base_link` 的安装外参，是两项不同标定。[S2][S5a]

**时间。**逐点时间用于运动畸变处理；雷达／IMU 同步、雷达／相机同步，以及与小车／其他车辆的时间对齐，需要分别核实。网络能收到 UDP 不能证明 ROS 时间已经一致。记录原始数据时应保留足以恢复这些时间关系的信息。

**地图与控制坐标。**SLAM 启动坐标通常不等于 UWB 坐标。接入时需要估计地图与 UWB 的旋转、平移，再把传感器位姿转换为底盘位姿。回环可能修正全局位置，控制应使用连续的局部里程计，并通过地图到里程计的变换表达全局校正，避免直接把回环跳变当作瞬时运动。

**当前控制接口。**[现有编队说明](formation-algorithm.md)明确记录：控制器的 X、Y 来自 `/ugvN/uwb/pose`，航向来自 `/ugvN/odom_combined`。引入 SLAM 的航向需要明确修改或配置相应输入；只替换位置话题不能同时替换航向。应继续保留定位有效性和数据时效判断。

**与 UWB 的关系。**UWB 可提供全局参考或重定位初值，LIO 提供连续相对运动，两者有互补性。当前 UWB EKF 已融合轮速／IMU，进一步融合时需处理重复使用同一观测带来的相关性。现有报告的毫米级静态散布衡量稳定性，不能直接作为 SLAM 绝对误差的比较基准。

**多车范围。**第一阶段可将 MID360 用于单车定位与地图构建。让其他车辆在同一地图获得各自位姿，还需要各车观测或明确的协同定位机制；一台雷达的自身里程计只直接描述承载它的车辆。

## 6. 建议的验证顺序

1. **先录制可重复数据。**包含静止、直行、转弯、闭环路径、长走廊和人员走动，记录原始点云、内置 IMU、轮速、UWB 和时间信息。使用独立测量／动捕作为真值时明确同步与坐标对齐；只有 UWB 对照时不宣称得到绝对精度。
2. **运行 FAST-LIO2 基线。**先按典型 10 Hz 点云输入，在独立工作空间验证 driver2 消息、时间、外参，再测 Nano 的运行负载。可从每帧约数千至一万有效点开始调试，这只是降采样试验范围，不是已验证最优值。
3. **同包比较完整 SLAM。**在兼容主机评估 Voxel-SLAM；若具备新系统／GPU 平台，再加入 GLIM。对齐输入数据、点数、地图分辨率与运行条件，并报告各自设置。
4. **有证据后再扩展。**如果主要问题是剧烈运动，加入 Point-LIO；如果主要问题是激光几何退化且可以提供同步相机，加入 FAST-LIVO2。
5. **最后接入编队定位。**先观察 SLAM 与现有定位的差异，再完成坐标与航向输入对齐，验证延迟、跳变及失效恢复。

| 验证维度 | 应记录的指标 |
|---|---|
| 实时性 | 点云处理与端到端位姿时延 P50／P95／P99、积压、掉帧；10 Hz 输入须持续跟得上 100 ms 周期 |
| 计算负载 | CPU、RSS 内存、温度、降频、长时间地图增长；给控制与通信保留实测余量 |
| 定位质量 | 有真值时统计 ATE／RPE，分别报告位置和航向；无真值时报告闭环误差与重复性 |
| 稳定性 | 静止漂移、急转弯、长走廊、动态遮挡、暂停／断流后的行为 |
| 地图能力 | 墙面重影、闭环前后的一致性、误回环、重启后定位成功率与耗时 |
| 编队可用性 | 公共坐标系中的位姿时效、航向一致性、回环或失效恢复时的最大跳变 |

**当前可执行的下一步是 driver2 + FAST-LIO2 的单车基线验证。**如果目标明确要求完整回环建图，Voxel-SLAM 应同时进入主机离线对比，而不是只检查 FAST-LIO2 的点云显示。

## 7. 一手资料与核查版本

- **[S1]** [Livox MID360 官方规格](https://www.livoxtech.com/mid-360/specs)。
- **[S2]** [FAST-LIO 官方仓库](https://github.com/hku-mars/FAST_LIO)；[FAST-LIO2 论文](https://arxiv.org/abs/2107.06829)。
- **[S2a]** [FAST-LIO MID360 参数](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/config/mid360.yaml)；[启动文件](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/launch/mapping_mid360.launch)。
- **[S2b]** [FAST-LIO 预处理接口](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/preprocess.h)；[构建依赖](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/CMakeLists.txt)。
- **[S3]** [Point-LIO 官方说明，核查默认分支](https://github.com/hku-mars/Point-LIO/tree/4b86a469eb5572e70ed575af25b5f15dd06e8e3c)；[Point-LIO 论文 DOI](https://doi.org/10.1002/aisy.202200459)。
- **[S4]** [Voxel-SLAM 官方仓库](https://github.com/hku-mars/Voxel-SLAM)；[论文](https://arxiv.org/abs/2410.08935)；[MID360 配置](https://github.com/hku-mars/Voxel-SLAM/blob/70fc8a28d63823d5989ff184daeea0787b672398/VoxelSLAM/config/mid360.yaml)。
- **[S5]** [GLIM 官方仓库](https://github.com/koide3/glim)；[当前安装条件](https://github.com/koide3/glim/blob/2262aafa2369acff6afab2268a7743aec7a7fa49/docs/installation.md)；[论文](https://arxiv.org/abs/2407.10344)。
- **[S5a]** [GLIM 参数说明](https://github.com/koide3/glim/blob/2262aafa2369acff6afab2268a7743aec7a7fa49/docs/parameters.md)。
- **[S6]** [FAST-LIVO2 官方仓库](https://github.com/hku-mars/FAST-LIVO2)；[论文](https://arxiv.org/abs/2408.14035)。
- **[S6a]** [FAST-LIVO2 on Resource-Constrained Platforms 论文](https://arxiv.org/abs/2501.13876)。
- **[S7]** [LIO-SAM 官方仓库及传感器要求](https://github.com/TixiaoShan/LIO-SAM)；[论文](https://arxiv.org/abs/2007.00258)。
- **[S8]** [Livox ROS Driver2 官方文档](https://github.com/Livox-SDK/livox_ros_driver2)。
- **[S9]** [FAST_LIO_SLAM：FAST-LIO2 + SC-PGO](https://github.com/gisbi-kim/FAST_LIO_SLAM)。
- **[S10]** [FAST_LIO_LOCALIZATION：已有地图定位](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)。

核心源码核查版本：FAST-LIO `7cc4175`；Point-LIO 默认分支 `point-lio-with-grid-map`，`4b86a46`；Voxel-SLAM `70fc8a2`；GLIM `2262aaf`；FAST-LIVO2 `0d2c034`。上游后续版本的依赖与支持范围可能改变，部署时应固定并复核实际版本。
