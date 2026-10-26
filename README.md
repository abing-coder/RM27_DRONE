# RM27 DRONE

面向 **RoboMaster 2027 超级对抗赛空中机器人**的视觉定位与自主导航系统。基于 ROS 2 Humble、Gazebo Classic 11 和 C++，用于先在仿真中验证感知、定位、规划与飞行控制，再向实机迁移。

赛事中的空中系统由空中机器人与机库组成，承担攻击和侦察任务。本项目当前聚焦**单机起飞、导航、识别敌方普通装甲、冲击、撤离、返航与视觉降落**，不包含完整多机调度与赛事机库通信。

[仿真预览](#仿真预览) · [开源来源与致谢](#开源来源与致谢) · [快速开始](#快速开始) · [系统架构](#系统架构) · [运行模式](#运行模式) · [在线规划](docs/navigation.md) · [视觉识别](docs/vision.md) · [仿真场景](docs/simulation.md)

## 仿真预览

以下为新赛季 STEP 场地在 Gazebo 中的实际渲染，以及系统运行截图。运行示例使用 `profile:=gt` 真值定位调试模式；地图、装甲与图片来源见[图片说明](docs/images/README.md)。

**新赛季场地全景**

![RMUC 2027 STEP 场地全景，包含红蓝基地、前哨站、中央能量机关及机库](docs/images/field-overview.png)

保留 CAD 场地布局与原生装甲，补充标准小装甲贴纸、红蓝灯条和接触面。能量机关与前哨站保持静止。

**自主规划与飞行可视化**

![RViz 中的巡航规划、场地障碍物、飞机姿态、深度点云与相机预览](docs/images/simulation-rviz.png)

此截图采集于 2026-10-22，展示静态规划版本的飞行可视化；当前默认已升级为 EGO 在线局部规划。棕色方块是 `Field` 规划障碍物，彩色点是深度相机点云；它们是不同的数据来源。`Odom` 仅显示最新姿态，飞行路径单独显示。

**机载相机与视觉推理预览**

![Gazebo 相机图像及 YOLO OpenVINO 推理状态、FAST 预览角点](docs/images/simulation-vision.png)

该图保留为旧权重的历史对照，当前模型示例见下图。绿色点为 FAST 预览角点。图中 `candidates=0`、`poses=0` 表示该帧没有有效装甲检测；`PREVIEW ONLY` 表示结果仅供查看，不进入视觉导航。当前模型的验证条件与结果见[视觉文档](docs/vision.md)。

| 红方基地标准小装甲 | 蓝方前哨站标准小装甲 |
|---|---|
| ![红方基地装甲贴纸、红色灯条与 CAD 外壳近景](docs/images/armor-red-base.png) | ![蓝方前哨站装甲贴纸、蓝色灯条与 CAD 外壳近景](docs/images/armor-blue-outpost.png) |

**当前 INT8 模型的实际预测（独立验证视角）**

| 红方基地 | 蓝方前哨站 |
|---|---|
| ![红基地 YOLO 检测框与四关键点](docs/images/sim-detection-red-base.jpg) | ![蓝前哨站 YOLO 检测框与四关键点](docs/images/sim-detection-blue-outpost.jpg) |

## 开源来源与致谢

本系统使用了下列开源代码、模型和推理约定。感谢原作者与维护团队；系统集成和场景适配基于这些已有工作完成。

| 来源与作者 | 本项目使用的内容 | 引用方式与许可说明 |
|---|---|---|
| 用户提供的 RMUC 2027 V1.0.0 STEP 场地与能量单元 | 新赛季完整场地几何、面颜色、原生装甲外壳和道具 | 保留原作者权利；原始文件未提供下载链接或独立许可，不按项目 MIT 重许可。见[CAD 来源与转换](ws/src/rm_field_gazebo/models/rmuc2027_official/SOURCE.md) |
| [EGO-Planner / EGO-Swarm — ZJU-FAST-Lab](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)，Xin Zhou、Jiangchao Zhu、Hongyu Zhou、Chao Xu、Fei Gao 及贡献者 | ROS 2 分支的 B-spline 优化核心、局部搜索与多项式初始化；本项目新增在线点云映射和任务适配 | 固定提交 `23a8d5a191711dd65633df689bd00f55d4dea8f9`；GPL-3.0，保留原始声明和[集成来源说明](ws/src/rm_ego_planner/SOURCE.md) |
| [OpenVINS — rpng/open_vins](https://github.com/rpng/open_vins)，Patrick Geneva、Guoquan Huang、Kevin Eckenhoff 及贡献者 | 单目/双目视觉惯性里程计 | 直接包含为 `ws/src/open_vins` 子模块，固定提交 `69488123ed9362dd44b6f28e7f4680abbff1442b`；GPL-3.0-or-later |
| [Daedalus — bevy_robomaster_simulator](https://github.com/Blackjack200/bevy_robomaster_simulator)，Blackjack200 / Actor&Thinker | 旧图曾使用的能量机关 `POWER.glb`、科技核心与机械臂 `TECH_CORE.glb` | 导入并转换为场景网格，固定提交 `43681255bc5415cdc15797d54fe7478062af42ed`；保留原模型、AGPL-3.0 许可及[转换说明](ws/src/rm_field_gazebo/models/rm_field/meshes/source/README.md) |
| RMUC 2026 场地网格，模型元数据署名 **pb2025 / ITL_Hero_Shoot**；相关项目：[北极熊 rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)、[ITL_Hero_Shoot](https://github.com/qwq9966qwq/ITL_Hero_Shoot) | 旧图使用过的场地外观原网格及配色衍生网格；当前场地改用 STEP | 原网格保存在 `ws/src/rm_field_gazebo/models/rmuc_2026`；沿用模型上游许可。北极熊工程的 Apache-2.0 声明与具体模型的授权范围分别记录，见[详细来源](docs/credits.md) |
| [RM27 本地来源工程](https://github.com/abing-coder/RM27)，所用视觉代码保留 **TongjiSuperPower** 版权；对应视觉框架参考 [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) | `yolo11.xml/.bin` 权重副本、YOLO11 预处理/输出解码约定及装甲类别表 | 权重由用户提供的本地工程复制，ROS 2 推理封装在本项目中实现；保留其代码 MIT 许可、版权声明和[文件哈希](ws/src/rm_drone_vision/models/SOURCE.md)。权重的原始训练来源与独立授权尚未核实，不将其归为本项目训练成果 |

默认仿真权重基于 [Ultralytics YOLO11n-pose](https://github.com/ultralytics/ultralytics/tree/v8.3.0) 微调，按 **AGPL-3.0** 提供，使用 [NNCF](https://github.com/openvinotoolkit/nncf) 量化；[训练来源、配方、哈希及许可](ws/src/rm_drone_vision/models/sim_armor/SOURCE.md)均随附。OpenVINS 静止启动修复通过[项目补丁](patches/README.md)提供，`build.sh` 自动检查并应用。

系统还依赖 [ROS 2](https://github.com/ros2/ros2)、[Gazebo Classic](https://github.com/gazebosim/gazebo-classic)、[OpenCV](https://github.com/opencv/opencv)、[OpenVINO](https://github.com/openvinotoolkit/openvino)、[Eigen](https://gitlab.com/libeigen/eigen) 和 [yaml-cpp](https://github.com/jbeder/yaml-cpp)。运行依赖与直接导入的代码、模型分别列在[完整来源与许可说明](docs/credits.md)中。

装甲贴图提取自 DJI《RoboMaster 2027 机器人制作规范手册》，版权归 DJI，来源页码见[场景文档](docs/simulation.md)。README 的组织方式参考 [chenjunnn/rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim)，此项为文档结构参考，未导入其自瞄模块代码。轨迹规划、控制和滤波的方法出处见[算法文献](docs/credits.md#自写代码所依据的方法)。

## 系统架构

```mermaid
flowchart LR
    RGB[前视 / 斜视彩色相机] --> DET[YOLO11 / 灯条识别]
    DET --> PNP[IPPE PnP 装甲位姿]
    STEREO[单目 / 双目相机] --> VIO[OpenVINS]
    VIO --> EST[ESKF 状态估计]
    IMU[IMU、下视 ToF 与气压计] --> EST
    GT[Gazebo 真值：调试模式] --> EST
    EST --> PNP
    EST --> MISSION[任务状态机]
    PNP --> MISSION
    DOWN[下视相机与原生 H 平台] --> LAND[视觉对中与接地检测]
    LAND --> MISSION
    LAND --> EST
    PNP --> EST
    DEPTH --> PNP
    REF[裁判阶段与机库放行] --> MISSION
    DEPTH[深度点云] --> MAP[在线占据地图与场地先验]
    MAP --> PLAN[A* 导向与 EGO B-spline]
    MISSION --> PLAN
    PLAN --> MISSION
    MISSION --> CTRL[位置控制器]
    EST --> CTRL
    CTRL --> FC[姿态与推力接口]
    FC --> SIM[仿真飞控与四旋翼动力学]
```

| 模块 | 当前实现 |
|---|---|
| 视觉感知 | YOLO11 敌方基地/前哨站关键点识别，可切换传统灯条；共用 PnP 与图像时刻位姿转换 |
| 状态估计 | OpenVINS + ESKF、IMU、ToF和气压高度，固定装甲/H平台地图校正；支持独立真值对照 |
| 轨迹规划 | 深度点云在线融合、A* 导向、EGO B-spline 局部优化与重规划；保留静态 minimum-snap 对照 |
| 飞行控制 | 位置环输出姿态和归一化推力，仿真内环跟踪；另有可选 PX4 桥 |
| 任务与约束 | 起飞、巡航、搜索、冲击、撤离、返航、视觉对中与接地确认；裁判放行、估计健康检查、电子围栏和急停 |
| 场景 | 2027 STEP 场地，基地、固定前哨站、静止能量机关、静止飞镖靶与机库 |

`map` 使用 ENU，原点位于场地中心，+x 指向蓝方。视觉输入是彩色相机，VIO 使用独立的单目/双目灰度流。

## 快速开始

环境：Ubuntu 22.04 / WSL2、ROS 2 Humble、Gazebo Classic 11、OpenVINO C++ Runtime。当前提供的 YOLO 后端使用 OpenVINO CPU 推理。

### 1. 准备依赖

在已有 ROS 2 Humble 的环境中：

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-rqt-image-view ros-humble-image-transport-plugins \
  ros-humble-cv-bridge ros-humble-message-filters \
  ros-humble-tf2-geometry-msgs python3-colcon-common-extensions \
  python3-numpy python3-pil libopencv-dev libeigen3-dev libyaml-cpp-dev \
  libceres-dev libboost-all-dev xvfb mesa-utils
```

安装 [OpenVINO C++ Runtime](https://docs.openvino.ai/2026/get-started/install-openvino.html) 并加载其环境；已有安装时使用对应的 `setupvars.sh`。CMake 需要能找到 `OpenVINOConfig.cmake`，配置方法见[视觉接入文档](docs/vision.md#依赖与构建)。

### 2. 初始化并编译

以下命令从仓库根目录执行：

```bash
git submodule update --init --recursive
./build.sh --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip ov_eval ov_data
source ws/install/setup.bash
```

`ov_eval`、`ov_data` 为 OpenVINS 评测工具及示例数据。场景和 YOLO11 模型已包含在项目中，无需每次启动时重新生成或复制。

### 3. 启动查看

```bash
./sim.sh auto_start:=false
```

打开 Gazebo、RViz 与视觉调试窗口，保持准备状态。

默认使用用户提供的 **RMUC 2027 V1.0.0 STEP 新场地**，直接运行 `./sim.sh` 即可加载，无需额外指定地图。旧地图已停用，旧版世界文件和导航地图已移除。

新版保留完整 CAD 场景及原生装甲，补充十二块标准装甲的贴纸、灯条和接触面；基地保持 CAD 闭合护甲姿态。机库台面高 1.6 m，出生点、测高、规划与装甲坐标来自同一版本资源。能量机关与飞镖靶保持静止，旧图道具运动接口不在当前场地启用。具体范围与生成方法见[新赛季 STEP 地图](docs/simulation.md#新赛季-step-地图)。

默认使用 EGO 在线规划，深度点云为 20 Hz；真值调试使用异步仿真，按采样时刻 TF 和新鲜度检查接收点云。视觉定位默认锁步，GT默认异步；保留图像、点云与位姿的采样时间检查；静态规划对照使用 `planner:=astar`。默认使用本项目仿真图微调的 YOLO11n 四关键点 INT8 模型，通过 OpenVINO CPU 推理；旧 RM27 权重保留用于对照；传统灯条对照用 `detector:=traditional`。启动器自动检测硬件 OpenGL，支持时让 Gazebo 相机使用 GPU 渲染；无硬件显示时使用 Xvfb 软件模式。默认保留左右目图像、彩色图像、深度图与点云，RViz 深度显示直接使用这些数据；下视 ToF 始终保留。启动仿真前需要完成上面的编译。

自动任务与视觉验收：

```bash
./sim.sh                          # 真值定位，自动执行仿真任务
./sim.sh mono allow_blind_attack:=false   # 单目 VIO，冲击前要求视觉确认
./sim.sh stereo allow_blind_attack:=false # 双目 VIO，冲击前要求视觉确认
```

默认规划器为 EGO：按图像时间将深度点云融合到地图，新障碍触发局部重规划；深度断流、轨迹过期或近距障碍触发刹停。任务节点是唯一的控制设定点发布者。场地 CAD 作为可选静态先验，详细接口、参数和限制见[在线建图与 EGO 规划](docs/navigation.md)。

默认仿真裁判已设为比赛开始和机库放行。默认要求视觉确认；仅显式设置 `allow_blind_attack:=true` 才按地图已知目标盲冲击；**验证 YOLO 是否实际参与任务时，应显式使用 `allow_blind_attack:=false`**。模型是否能识别某个仿真视角需通过图像验证，不能用默认任务飞完作为识别成功的依据。

## 运行模式

| 用法 | 定位来源 | 适用场景 |
|---|---|---|
| `./sim.sh` / `./sim.sh gt` | 仿真真值，关闭 VIO | 控制、轨迹与场景调试 |
| `./sim.sh mono` | 单目 OpenVINS + ESKF | 单目视觉惯性定位验收 |
| `./sim.sh stereo` | 双目 OpenVINS + ESKF | 双目视觉惯性定位验收 |
| `./sim.sh off` | ESKF，不启 VIO、不用真值 | IMU / ToF 与估计器调试 |

识别后端与定位模式独立：`mono` / `stereo` 指 VIO，不代表 YOLO 开关。视觉模式要求健康的 VIO 估计；与所选模式矛盾的真值参数会使启动失败。

| 参数 | 用途 | 默认 |
|---|---|---|
| `team:=red/blue` | 本方队伍、机库起点与敌方颜色 | `red` |
| `planner:=ego/astar` | EGO 在线局部规划 / 原静态 A* + minimum-snap 对照 | `ego` |
| `map_prior:=true/false` | 是否使用巡航区域 CAD 先验；关闭后未知区域限制、机库起飞配置和规则边界仍保留 | `true` |
| `depth_rate:=auto` | 深度采样率，EGO 20 Hz、静态 A* 5 Hz，可显式设为 1–30 Hz | `auto` |
| `depth_topic:=/drone/depth/points` | 在线建图点云输入话题 | `/drone/depth/points` |
| `target:=auto` | 自动选择，或指定如 `blue_outpost_armor_0` | `auto` |
| `field:=official` | 2027 V1.0.0 STEP 场地，唯一地图，无需手动指定 | `official` |
| `detector:=yolo/traditional` | OpenVINO YOLO11 四关键点 / 传统灯条识别 | `yolo` |
| `render_backend:=auto/native/software` | 自动检测硬件 OpenGL / 桌面渲染 / Xvfb 软件渲染 | `auto` |
| `gui_render_backend:=auto/native/software` | Gazebo 窗口渲染，`auto` 跟随相机后端；可单独切换软件兼容模式 | `auto` |
| `show_features:=true/false` | 预览绿色 FAST 角点，与 YOLO 关键点和 VIO 跟踪无关 | `true` |
| `show_yolo_rejected:=true/false` | 无有效候选时，显示最高分被拒绝的真实网络预测（橙色虚线，仅预览） | `true` |
| `use_depth:=true/false` | 深度图和点云数据源开关；不影响下视 ToF | `true` |
| `auto_start:=false` | 等待手动任务启动 | `true` |
| `lockstep:=auto/true/false` | 相机与物理锁步；`auto` 在视觉定位时锁步，GT时异步 | `auto` |
| `allow_blind_attack:=false` | 冲击前必须获得视觉确认 | `false` |
| `gui:=false rviz:=false imshow:=false` | 关闭图形界面 | 均为 `true` |

例如，仅启用后端服务：

```bash
./sim.sh gui:=false rviz:=false imshow:=false auto_start:=false
```

手动启动或急停，在另一个加载 `ws/install/setup.bash` 的终端执行：

```bash
ros2 topic pub --once /drone/mission/start std_msgs/msg/Empty '{}'
ros2 topic pub --once /drone/estop std_msgs/msg/Bool '{data: true}'
```

自动起飞需裁判许可、机库放行、有效里程计和定位初始化；EGO 还要求新鲜深度、采样时刻 TF 及起飞区域无在线近距障碍。满足条件后连续等待配置的仿真时间。终端每 5 秒墙上时间输出一次“等待起飞”的具体原因与仿真时间；同一原因也在 `/drone/mission_state.info` 中。正常起飞会出现 `IDLE → ARMING → TAKEOFF`。起飞轨迹结束后还需实际位置误差小于 5 cm、速度小于 0.05 m/s，且当前位置已离开膨胀障碍区，才进入规划。`auto_start` 仅接受 `true/false`，拼写错误会直接报错。起飞轨迹结束后，会等待飞机接近悬停位置、速度稳定并离开机库的避障膨胀区，再开始巡航规划。

真值模式中启用 YOLO 不会改变定位来源。只检查识别画面时可用 `auto_start:=false`；调度取决于定位模式，禁止盲冲击不改变调度。仿真暂停或等待渲染时不累计墙钟断流；恢复后仍按有效采样时间检查超时，实机接收看门狗继续使用单调时钟。

两种识别后端都求解 IPPE PnP，`/drone/armors` 中的 `pose_camera` 表示装甲在相机系的位置，`has_map_pose=true` 时才有可用于导航的 `pose_map`。imshow 显示 `yolo/OpenVINO`、推理帧号、耗时、关键点和相机输入帧率。旧帧仍运行推理并显示橙色 `PREVIEW ONLY`，不将旧帧用于定位。查看实时 YOLO 画面可执行 `./sim.sh detector:=yolo auto_start:=false`，各字段与排查方法见[视觉文档](docs/vision.md)。

Gazebo 服务端 `gzserver` 负责仿真，`gzclient` 负责场景窗口；服务端在运行不等于窗口已显示。若窗口黑屏，可执行 `./sim.sh gui_render_backend:=software auto_start:=false` 单独切换窗口渲染。绿色圆点是预览角点；有效 YOLO 检测用蓝色实线框和黄色四关键点，PnP 通过后叠加绿色四边形。橙色虚线 `PREVIEW ... rejected` 表示网络预测被过滤，不进入定位；没有真实预测时不会显示框。

## 代码与接口

| 功能包 | 职责 |
|---|---|
| `rm_drone_msgs` | 装甲、轨迹、姿态设定、裁判和任务消息 |
| `rm_drone_vision` | YOLO11 / 灯条识别、PnP、视觉调试图像 |
| `rm_drone_estimator` | ESKF、OpenVINS 标定与运行配置 |
| `rm_drone_planner` | 全局导向、轨迹校验、任务状态机和模拟裁判 |
| `rm_ego_planner` | EGO 优化核心、在线深度占据融合、局部重规划和健康检查 |
| `rm_drone_control` | 位置控制、仿真飞控及可选 PX4 桥 |
| `rm_drone_description` | 无人机模型与传感器定义 |
| `rm_field_gazebo` | 场地模型、材质、世界和规划地图生成器 |
| `rm_drone_gazebo_plugins` | 动力学、装甲冲击与场地机构插件 |
| `rm_drone_bringup` | 系统启动与 RViz 配置 |
| `open_vins` | 上游 VIO 子模块，固定版本见[来源说明](docs/credits.md) |

| ROS 接口 | 内容 |
|---|---|
| `/drone/color/image_raw`、`/drone/attack/image_raw` | 前视彩色与基地斜视识别图像，各配套 camera_info |
| `/drone/armors` | 装甲类别、关键点、相机位姿及有效时的 map 位姿 |
| `/drone/odom` | 估计器输出 |
| `/drone/vision/debug_image` | 检测框、关键点、PnP 结果和推理耗时 |
| `/drone/ego/status`、`/drone/ego/occupancy` | 在线地图健康、阻塞状态与观测障碍 |
| `/drone/mission_state` | 任务阶段与状态 |
| `/drone/setpoint`、`/drone/attitude_target` | 轨迹和姿态控制接口 |

## 当前边界与实机迁移

规则依据是本地 2027 V1.0.0 两份 PDF。无人机冲击对象仅为敌方基地和前哨站普通装甲；飞镖移动靶、能量机关和机库不属于其冲击目标。场景采用用户提供的 2027 STEP 几何，碰撞简化、规划体素和仿真参数仍需实测验证，不能作为实机场地测量基准。

当前是单机仿真开发系统。尚未实现完整四机协作、侦察回传、实机机库通信和正式裁判协议，也未完成实机飞行验证。当前默认仿真模型已在独立装甲实例上验证；0.7 门限下精确率约 98.8%、召回率约 79.1%，INT8 推理中位耗时约 17.5 ms（i9-14900HX，2 线程）。这不是香橙派性能或实机识别精度。完整测试范围、结果与尚未覆盖的条件见[仿真验收](docs/sim-acceptance.md)，视觉验收应关闭盲冲击。

迁移实机需要替换相机/IMU/ToF 驱动、用 Kalibr 标定外参、验证机载推理性能，并配置 PX4/uXRCE-DDS 及 `px4_msgs`。`sim.sh` 仅用于仿真。

## 文档与许可

- [在线建图、EGO 局部规划与保护逻辑](docs/navigation.md)
- [视觉推理、模型类别与配置](docs/vision.md)
- [地图格式、配色、场地生成与道具调试](docs/simulation.md)
- [第三方模型、算法引用与许可证](docs/credits.md)

除 `rm_ego_planner`（链接上游 EGO 算法，按 GPL-3.0 分发）外，本仓库新增的系统集成与其他 `rm_*` 实现使用 [MIT](LICENSE)；上述第三方代码、模型、权重与纹理的版权及许可单独保留，根目录 MIT 不替代其原有声明。
