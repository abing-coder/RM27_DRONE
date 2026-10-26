# 来源、方法与许可证

[返回系统介绍](../README.md)

除按 GPL-3.0 分发的 `rm_ego_planner` 外，本仓库自研的 `rm_*` 功能包以 MIT 许可发布，全文见 [LICENSE](../LICENSE)。下面这些代码和模型来自其他项目，使用和再分发时保留它们自己的许可证。

## 直接包含的代码和模型

| 内容 | 来源 | 许可证 | 在本仓库中的位置 |
|---|---|---|---|
| 用户提供的 RMUC 2027 V1.0.0 场地与能量单元 STEP | 用户提供；文件头记录 Creo Parametric，未附官方下载地址 | 原作者保留权利；未附独立许可，不按根目录 MIT 重许可 | `models/rmuc2027_official`、`models/rmuc2027_energy_unit`；[来源、哈希及转换](../ws/src/rm_field_gazebo/models/rmuc2027_official/SOURCE.md) |
| EGO-Planner / EGO-Swarm | [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)，固定 ROS 2 提交 `23a8d5a191711dd65633df689bd00f55d4dea8f9` | GPL-3.0；保留文件内 Fast-Planner 等原始声明 | `ws/src/rm_ego_planner/vendor` 与 GPL 运行适配包；[完整来源及修改](../ws/src/rm_ego_planner/SOURCE.md) |
| OpenVINS（MSCKF 视觉惯性估计器） | [rpng/open_vins](https://github.com/rpng/open_vins)，Patrick Geneva、Guoquan Huang、Kevin Eckenhoff 及贡献者 | GPL-3.0-or-later | 子模块 `ws/src/open_vins` |
| RMUC 2026 场地外观网格 | 模型元数据作者为 pb2025 / ITL_Hero_Shoot。相关工程：[北极熊 rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)、[ITL_Hero_Shoot](https://github.com/qwq9966qwq/ITL_Hero_Shoot) | 随上游模型；北极熊仿真器为 Apache-2.0 | 原文件：`ws/src/rm_field_gazebo/models/rmuc_2026`；分区配色衍生网格：`models/rm_field/meshes/field_*` |
| 详细能量机关及科技核心几何 | Blackjack200 / Actor&Thinker，[Daedalus](https://github.com/Blackjack200/bevy_robomaster_simulator)，固定提交 `43681255bc5415cdc15797d54fe7478062af42ed` | AGPL-3.0；原始模型及完整许可一并保留，派生几何不按根目录 MIT 重许可 | `ws/src/rm_field_gazebo/models/rm_field/meshes/source/` 及生成的 `energy_*.stl`（不含自制 `energy_ring.stl`、`energy_plinth.stl`），以及 `technology_core*.stl` |
| YOLO11 权重副本与解码约定 | 直接来源：[abing-coder/RM27](https://github.com/abing-coder/RM27) 的本地副本；相关代码保留 TongjiSuperPower 版权，对应视觉框架参考 [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) | 随附代码 MIT 许可已保留；权重的原始训练来源及独立授权尚未核实 | `ws/src/rm_drone_vision/models/`；来源文件、哈希与接入范围见 [SOURCE.md](../ws/src/rm_drone_vision/models/SOURCE.md) |

新增仿真模型基于 Ultralytics v8.3.0 的 YOLO11n-pose（AGPL-3.0）微调，使用 OpenVINO/NNCF 2.19.0（Apache-2.0）量化。权重不按根目录 MIT 重许可，训练与部署文件哈希、完整许可见 [sim_armor/SOURCE.md](../ws/src/rm_drone_vision/models/sim_armor/SOURCE.md)。OpenVINS 的静止 ZUPT 启动与异步回调修复见 [patches](../patches/README.md)，不修改其上游版权声明。

OpenVINS 子模块固定提交为 `69488123ed9362dd44b6f28e7f4680abbff1442b`。

OpenVINS 的论文：Patrick Geneva, Kevin Eckenhoff, Woosik Lee, Yulin Yang, Guoquan Huang, “OpenVINS: A Research Platform for Visual-Inertial Estimation”, ICRA 2020。滤波器形式来自 Anastasios I. Mourikis, Stergios I. Roumeliotis, “A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation”, ICRA 2007。

OpenVINS 是 GPL-3.0。把本仓库与 OpenVINS 链接后一起分发时，组合作品需要遵守 GPL-3.0；未与 OpenVINS 链接的 `rm_*` 源码仍按 MIT 使用。OpenVINS 的许可证原文在子模块的 `LICENSE`，请不要删掉子模块里的版权声明。

旧版场地外观网格只做显示，碰撞体和规划地图由 `gen_field.py` 生成。新版 STEP 场地通过 `gen_official_field.py` 生成带面颜色的 DAE、静态 STL 碰撞、CAD 表面占据地图与精确测高三角形。转换工具离线使用 Open CASCADE / cadquery-ocp、VTK、NumPy 与 PyYAML，运行仿真不需要这些 CAD 依赖。

场地原网格的作者署名以随附 `model.config` 为依据；上述两个相关工程用于记录传播来源，未据其工程级许可推断所有模型文件均采用相同许可。分区配色不改变原几何来源与作者署名。

装甲图案来自 DJI 制作规范手册，版权归 DJI，具体手册版本与提取页码见[场景文档](simulation.md#装甲与场地道具)。这些官方图案与手册不标为本项目原创，也不按根目录 MIT 重新授权。

## 编译和运行时依赖（不把源码放进本仓库）

| 项目 | 用途 | 许可证 |
|---|---|---|
| [ROS 2 Humble](https://docs.ros.org/en/humble/) | 通信、launch、tf | Apache-2.0 |
| [Gazebo Classic 11](https://classic.gazebosim.org/) | 物理仿真与传感器 | Apache-2.0 |
| [OpenCV](https://opencv.org/) | 装甲灯条检测、PnP | Apache-2.0 |
| [OpenVINO](https://github.com/openvinotoolkit/openvino) | YOLO11 模型加载与 CPU 推理 | Apache-2.0，随安装包保留其第三方声明 |
| [Eigen](https://eigen.tuxfamily.org/) | 线性代数 | MPL-2.0 |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 读取场地与任务配置 | MIT |
| NumPy、Pillow | 生成场地纹理和世界文件 | BSD-3-Clause / HPND |

实机阶段才会用到、当前仓库不包含的工具：

| 项目 | 用途 | 许可证 |
|---|---|---|
| [PX4](https://github.com/PX4/PX4-Autopilot) 与 [px4_msgs](https://github.com/PX4/px4_msgs) | uXRCE-DDS 飞控桥 | BSD-3-Clause |
| [Kalibr](https://github.com/ethz-asl/kalibr) | 相机–IMU 标定，建议用其 Docker 镜像 | GPL-3.0 |

## 自写代码所依据的方法

这些是本仓库自己实现的，源码不来自下列论文或程序，写在这里是为了说明算法出处：

- 轨迹：7 次多项式 minimum-snap。Daniel Mellinger, Vijay Kumar, “Minimum Snap Trajectory Generation and Control for Quadrotors”, ICRA 2011。
- 位置控制：期望加速度对齐机体 z 轴，再限倾角。Taeyoung Lee, Melvin Leok, N. Harris McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3)”, CDC 2010。
- 误差状态卡尔曼滤波：Joan Solà, “Quaternion kinematics for the error-state Kalman filter”, arXiv:1711.02508。

## 本地 YOLO 模型与解码约定

YOLO11 权重复制自用户提供的本地 RM27 工程，其记录的 origin 为 [abing-coder/RM27](https://github.com/abing-coder/RM27)。来源文件、SHA-256 和随附 MIT 许可见 [models/SOURCE.md](../ws/src/rm_drone_vision/models/SOURCE.md)。解码类别表和预处理参考该工程中保留 TongjiSuperPower 版权的装甲识别实现；相关开源视觉框架为 [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)。记录本地来源不等于已确认这些权重最初由该团队训练。训练数据来源、训练指标与权重独立授权未在文件中说明；不将其标为本项目自研或已完成实机验证。

OpenVINO C++ Runtime 用于执行模型，作为外部依赖安装，项目不包含其源码。

## 文档组织参考

系统介绍、构建步骤和功能包说明的组织方式参考 [chenjunnn/rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim)，作者 Chen Jun，项目采用 MIT 许可。本次仅参考文档结构，未导入其识别或跟踪模块源码。

IMU时间戳适配使用Gazebo原生 `ImuSensor::ImuMessage()` 的测量快照，接口依据为
[Gazebo Classic IMU实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/sensors/ImuSensor.cc)，
并对照了[ROS IMU插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp)的噪声协方差约定。
两个上游项目均为Apache-2.0；本项目的小型适配插件自写，不修改系统安装的Gazebo文件。
