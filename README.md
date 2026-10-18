# RM27 空中机器人仿真

面向 RoboMaster 2027 机甲大师超级对抗赛（RMUC）空中机器人的算法仿真。在 Gazebo Classic 11 里放一块约 28 m × 15 m 的战场和一架小型四旋翼，把定位、规划、装甲识别和撞击串成一条能自动起飞的任务。

技术栈是 ROS 2 Humble 和 C++。仿真用来先把飞行和任务逻辑跑通；上实机时，估计器和位置环可以留在机载计算机上，内环换成 PX4。

一架飞机从机库起飞之后会做这些事：

1. 用前视相机的灯条配对和 PnP 找到敌方装甲，换算到场地坐标系。
2. 需要视觉惯性里程计时，用 [OpenVINS](https://github.com/rpng/open_vins) 做单目或双目 VIO；ESKF 再融合 IMU 和下视 ToF，并把位姿对齐到场地 `map`。默认任务直接用仿真真值，避免 VIO 还没收敛时飞机打转。
3. 在膨胀后的三维占据栅格里用 A* 找路，再用 7 次多项式 minimum-snap 生成轨迹。
4. 位置环输出姿态和归一化推力，仿真飞控按 PX4 offboard 姿态模式跟踪。
5. 任务状态机负责起飞、巡航、搜索、撞击、撤离和降落，并做电子围栏：软边界悬停，硬边界停桨。

机体按高校系列赛制作规范里的量级建模：约 150 g，桨保外包络约 140 mm。传感器有前视双目、400 Hz IMU、下视 ToF，以及仅仿真使用的深度相机。场地碰撞和规划地图按规则手册的文字描述做了近似布置，外观使用一份 2026 赛季场地网格。官方手册 PDF 不在仓库里。

## 仓库里有什么

| 路径 | 内容 |
|---|---|
| `sim.sh` | 一键启动仿真 |
| `ws/src/rm_*` | 本仓库的消息、场地、机体、飞控、估计、视觉、规划 |
| `ws/src/open_vins` | OpenVINS 子模块，GPL-3.0，见下文 |

克隆时带上子模块：

```bash
git clone --recursive <仓库地址>
cd RM27_DRONE
git submodule update --init --recursive
```

当前锁定的 OpenVINS 提交是 `69488123ed9362dd44b6f28e7f4680abbff1442b`（2025-11-30，`master`）。

## 依赖

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-rqt-image-view \
  ros-humble-image-transport-plugins ros-humble-cv-bridge \
  python3-colcon-common-extensions python3-numpy python3-pil \
  libyaml-cpp-dev xvfb
```

WSL 上 `gzserver` 放在 Xvfb 里跑，避免桌面 OpenGL 把物理仿真卡死。`gzclient` 仍用当前桌面显示。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/RM27_DRONE/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip ov_eval ov_data
source install/setup.bash
```

`ov_eval` 和 `ov_data` 是 OpenVINS 的评测工具和示例数据，飞仿真用不到。

场地世界、纹理和规划地图已经在仓库里。要按脚本里的尺寸重新生成：

```bash
python3 src/rm_field_gazebo/scripts/gen_field.py
```

## 启动仿真

在项目根目录：

```bash
./sim.sh                         # ESKF + 仿真真值，自动起飞
./sim.sh gt                      # 估计器直接用 Gazebo 真值
./sim.sh run_vio:=true vio:=mono # 再打开单目 OpenVINS
./sim.sh run_vio:=true vio:=stereo
./sim.sh gui:=false              # 其余参数原样传给 launch
```

`sim.sh` 和 `ws/src/rm_drone_bringup/launch/sim.launch.py` 里的 `GAZEBO_MODEL_DATABASE_URI` 目前写的是本机路径 `/home/abing/RM27_DRONE/...`。换机器后改成自己的仓库路径，否则 `gzclient` 会去连在线模型库。

窗口：Gazebo 3D、RViz（轨迹、规划和装甲）、识别调试窗口。

| 参数 | 含义 | launch 默认 |
|---|---|---|
| `team` | `red` / `blue`，机库起飞点与敌方颜色 | `red` |
| `target` | `auto`，或具体目标如 `blue_outpost`、`blue_base_armor_0` | `auto` |
| `estimator` | `gt` 真值 / `eskf` 融合 | `gt`（`sim.sh` 会改成 `eskf`） |
| `vio` | `mono` / `stereo` / `off` | `off` |
| `run_vio` | 是否启动 OpenVINS 节点 | `false` |
| `use_sim_truth` | ESKF 没有 VIO 时是否用仿真真值 | `true` |
| `use_depth` | 是否挂仿真深度相机 | `true` |
| `auto_start` | 启动后自动起飞 | `true` |
| `gui` / `rviz` / `imshow` | Gazebo 窗口、RViz、识别调试窗口 | `true` |

手动触发：

```bash
ros2 topic pub --once /drone/mission/start std_msgs/msg/Empty
ros2 topic pub --once /drone/estop std_msgs/msg/Bool "{data: true}"
```

## 功能包

| 包 | 作用 |
|---|---|
| `rm_drone_msgs` | 姿态设定、轨迹点、装甲、冲击、裁判、任务状态 |
| `rm_field_gazebo` | 战场世界、装甲灯条、前哨站、能量机关、规划用地图 |
| `rm_drone_gazebo_plugins` | 四旋翼动力学、装甲冲击、关节旋转 |
| `rm_drone_description` | 机体、双目、IMU、ToF、深度相机 |
| `rm_drone_control` | 仿真飞控、位置控制、PX4 桥（可选） |
| `rm_drone_estimator` | ESKF，以及单目/双目 OpenVINS 配置 |
| `rm_drone_vision` | 灯条装甲识别和 PnP |
| `rm_drone_planner` | A*、minimum-snap、电子围栏、任务状态机、裁判模拟 |
| `rm_drone_bringup` | 启动与 RViz |

坐标系：`map` 为场地 ENU，原点在战场中心，+x 指向蓝方。仿真飞控接口是 `AttitudeTarget`（姿态 + 归一化推力）。上实机时换成 `px4_bridge_node`（MAVLink / uXRCE-DDS），需要另行放入 `px4_msgs` 再编译。

## 开源引用与许可证

本仓库自研的 `rm_*` 功能包以 MIT 许可发布，全文见 [LICENSE](LICENSE)。下面这些代码和模型来自其他项目，使用和再分发时保留它们自己的许可证。

### 直接包含的代码和模型

| 内容 | 来源 | 许可证 | 在本仓库中的位置 |
|---|---|---|---|
| OpenVINS（MSCKF 视觉惯性估计器） | [rpng/open_vins](https://github.com/rpng/open_vins)，Patrick Geneva、Guoquan Huang、Kevin Eckenhoff 及贡献者 | GPL-3.0-or-later | 子模块 `ws/src/open_vins` |
| RMUC 2026 场地外观网格 | 模型元数据作者为 pb2025 / ITL_Hero_Shoot。相关工程：[北极熊 rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)、[ITL_Hero_Shoot](https://github.com/qwq9966qwq/ITL_Hero_Shoot) | 随上游模型；北极熊仿真器为 Apache-2.0 | `ws/src/rm_field_gazebo/models/rmuc_2026` |

OpenVINS 的论文：Patrick Geneva, Kevin Eckenhoff, Woosik Lee, Yulin Yang, Guoquan Huang, “OpenVINS: A Research Platform for Visual-Inertial Estimation”, ICRA 2020。滤波器形式来自 Anastasios I. Mourikis, Stergios I. Roumeliotis, “A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation”, ICRA 2007。

OpenVINS 是 GPL-3.0。把本仓库与 OpenVINS 链接后一起分发时，组合作品需要遵守 GPL-3.0；未与 OpenVINS 链接的 `rm_*` 源码仍按 MIT 使用。OpenVINS 的许可证原文在子模块的 `LICENSE`，请不要删掉子模块里的版权声明。

场地网格只做显示，不参与碰撞。碰撞体和规划地图由 `gen_field.py` 生成。

### 编译和运行时依赖（不把源码放进本仓库）

| 项目 | 用途 | 许可证 |
|---|---|---|
| [ROS 2 Humble](https://docs.ros.org/en/humble/) | 通信、launch、tf | Apache-2.0 |
| [Gazebo Classic 11](https://classic.gazebosim.org/) | 物理仿真与传感器 | Apache-2.0 |
| [OpenCV](https://opencv.org/) | 装甲灯条检测、PnP | Apache-2.0 |
| [Eigen](https://eigen.tuxfamily.org/) | 线性代数 | MPL-2.0 |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 读取场地与任务配置 | MIT |
| NumPy、Pillow | 生成场地纹理和世界文件 | BSD-3-Clause / HPND |

实机阶段才会用到、当前仓库不包含的工具：

| 项目 | 用途 | 许可证 |
|---|---|---|
| [PX4](https://github.com/PX4/PX4-Autopilot) 与 [px4_msgs](https://github.com/PX4/px4_msgs) | uXRCE-DDS 飞控桥 | BSD-3-Clause |
| [Kalibr](https://github.com/ethz-asl/kalibr) | 相机–IMU 标定，建议用其 Docker 镜像 | GPL-3.0 |

### 自写代码所依据的方法

这些是本仓库自己实现的，源码不来自下列论文或程序，写在这里是为了说明算法出处：

- 轨迹：7 次多项式 minimum-snap。Daniel Mellinger, Vijay Kumar, “Minimum Snap Trajectory Generation and Control for Quadrotors”, ICRA 2011。
- 位置控制：期望加速度对齐机体 z 轴，再限倾角。Taeyoung Lee, Melvin Leok, N. Harris McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3)”, CDC 2010。
- 误差状态卡尔曼滤波：Joan Solà, “Quaternion kinematics for the error-state Kalman filter”, arXiv:1711.02508。

## 实机还要准备的东西

- PX4 和 `px4_msgs`（uXRCE-DDS）
- 用 Kalibr 标定后，替换 `rm_drone_estimator` 里的 `kalibr_imucam_chain.yaml` / `kalibr_imu_chain.yaml`
- 机载计算机系统镜像（当前按 Radxa Zero 3W 这个量级预留）
