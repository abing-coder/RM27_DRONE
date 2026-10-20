# RM27 空中机器人仿真

面向 RoboMaster 2027 机甲大师超级对抗赛（RMUC）空中机器人的算法仿真。在 Gazebo Classic 11 里放一块约 28 m × 15 m 的战场和一架小型四旋翼，把定位、规划、装甲识别和撞击串成一条能自动起飞的任务。

规则依据为项目根目录的两份 2027 V1.0.0 本地 PDF。比赛规则印刷第 27 页规定：空中系统由空中机器人和机库组成，用于攻击与侦察；飞机全自动运行，遥控器用于急停，多机只能经机库间接通信。本项目当前实现单机的视觉定位、导航与装甲冲击流程，尚未实现四机调度、侦察信息回传和实机机库通信。无人机冲击目标为敌方基地/前哨站普通装甲；基地飞镖移动靶属于飞镖系统，能量机关和机库禁止无人机击打（规则第 64–65、134 页），不能因为场景中有同色灯效就把它们加入攻击目标。

技术栈是 ROS 2 Humble 和 C++。仿真用来先把飞行和任务逻辑跑通；上实机时，估计器和位置环可以留在机载计算机上，内环换成 PX4。

一架飞机从机库起飞之后会做这些事：

1. 用前视相机的灯条配对和 PnP 找到敌方装甲，换算到场地坐标系。
2. 需要视觉惯性里程计时，用 [OpenVINS](https://github.com/rpng/open_vins) 做单目或双目 VIO；ESKF 再融合 IMU 和下视 ToF，并把位姿对齐到场地 `map`。默认任务直接用仿真真值，避免 VIO 还没收敛时飞机打转。
3. 在膨胀后的三维占据栅格里用 A* 找路，再用 7 次多项式 minimum-snap 生成轨迹。
4. 位置环输出姿态和归一化推力，仿真飞控按 PX4 offboard 姿态模式跟踪。
5. 任务状态机负责起飞、巡航、搜索、撞击、撤离和降落，并做电子围栏：软边界悬停，硬边界停桨。

机体按高校系列赛制作规范里的量级建模：约 150 g，桨保外包络约 140 mm。传感器有前视双目、400 Hz IMU、下视 ToF，以及仅仿真使用的深度相机。场地碰撞和规划地图按规则手册做近似布置，静态外观以 2026 场地网格为基础；这不是官方 2027 CAD 的精确复刻。

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

## 场景文件与配色

地图采用 Gazebo SDF，整体入口是 `ws/src/rm_field_gazebo/worlds/rmuc2027.world`。无人机本体使用 `ws/src/rm_drone_description/urdf/rm_drone.urdf.xacro`；地图不使用 URDF。

| 后缀 | 用途 |
|---|---|
| `.world` / `.sdf` | 场景、模型、位姿、碰撞与插件定义 |
| `.stl` / `.dae` | 三维几何；带地面纹理的网格使用含 UV 的 DAE |
| `.glb` | 保留的上游详细模型原文件，用于重新转换 |
| `.png` / `.material` | 纹理及颜色、发光等 OGRE 材质 |
| `.yaml` | 规划地图、目标和仿真参数 |
| `.config` | Gazebo 模型名称、版本等元信息 |

统一灰色场地的版本已保存在提交 `7dadb55`，可用于对比。当前配色参考根目录比赛规则手册印刷第 29–30 页（PDF 第 30–31 页）的图 4-1、4-2、4-3：赛场以深灰地胶、较浅的灰色平台、黑色围挡和设备、红蓝及白色标线为主。木结构表面覆盖 PVC 地胶，不统一涂成棕色木纹。以下是仿真参考色，不是官方 RGB 标准；相机成像还会受到光照与曝光影响。

| 区域 | 参考色 |
|---|---|
| 地胶 | 深石墨灰 `#343638` |
| 高地、公路和平台上表面 | 中性灰 `#797D79` |
| 结构侧面 | 暗灰绿 `#585D55` |
| 基地、前哨站等设备外壳 | 石墨灰 `#303438` |
| 围挡 | 近黑 `#191C20` |
| 区域标线 | 白色边界，红蓝队伍色点缀 |

配色定义集中在 `ws/src/rm_field_gazebo/scripts/field_appearance.py`。生成器根据可见面的高度、朝向和设备部件范围拆分原场地网格，保留每个三角形的顶点，给地面和平台补充 UV 及低对比度 PVC 纹理；不再让整张 STL 共用单一灰材质。原 STL 没有材质语义，区域分配仍是近似。装甲图案、灯条材质和绿色引导灯保持独立，调场地颜色不会覆盖它们。

运行 `gen_field.py --skip-background-textures` 也会更新这些配色资源；该开关保留的是原有地面/木纹/围挡背景贴图。原始上游网格和碰撞地图不因配色而改变。

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

只更新装甲、材质、世界和地图，保留已有地面/木纹/围挡贴图：

```bash
python3 src/rm_field_gazebo/scripts/gen_field.py --skip-background-textures
```

装甲正面图案分别取自本地《机器人制作规范手册 V1.0.0（20261015）》印刷第 88–89 页（PDF 第 89–90 页）的附录图 1、3，版权归 DJI；提取为 `rm_armor_outpost.png`、`rm_armor_base.png`，不再使用占位数字。图案仅映射正面并保持原比例，灯条单独使用红蓝自发光材质。基地碰撞/装甲尺寸仍沿用当前 2026 网格的简化模型，贴纸来源不代表场地已经精确复刻 2027 规格。前哨站固定不转。

视觉和后续 YOLO 使用 `/drone/color/image_raw` 与对应 `camera_info`，双目 `stereo/*` 是 VIO 灰度输入。RGB/BGR 按消息 encoding 转换，不使用带调试叠加的 `vision/debug_image` 训练。`field_map.yaml` 中 `armor_classes` 固定为 `0=red_base`、`1=blue_base`、`2=red_outpost`、`3=blue_outpost`，每块装甲 name 是独立实例 ID。目标的 `position` 是灯条朝外可见面中心，`light_bar_corners` 为世界坐标下的左上、左下、右下、右上端点，与现有 PnP 一致；`normal` 保留倾角。这些几何真值只供离线标注，运行时仍由图像检测定位。当前未接入 YOLO 权重或批量采集器，检测框四角不能直接当 PnP 灯条端点使用。

仿真入口通过 `rm_field_gazebo/config/gazebo_ros.yaml` 将 `/clock` 发布频率设为 200 Hz，避免默认 10 Hz 时钟使相机/IMU 采样时间被误判为超前。视觉节点最多缓存 3 帧图像，等候位姿插值最多 100 ms；缺少对应位姿时只发布相机系检测，不伪造 map 位姿。

能量机关复用 [Daedalus 的 POWER.glb 详细模型](https://github.com/Blackjack200/bevy_robomaster_simulator/blob/43681255bc5415cdc15797d54fe7478062af42ed/assets/POWER.glb)，保留镂空灯臂、外框、分段灯带、靶面、R 标识与支撑结构；下方科技核心与机械臂复用同一项目的 `TECH_CORE.glb`，补建承托支脚的八边形底座。原始 GLB、固定提交号、AGPL v3 许可及转换说明保存在 `models/rm_field/meshes/source/`；生成器会应用节点层级变换、转换坐标轴，再导出 Gazebo Classic 能读取的 STL，不需要另装 Blender 或 Bevy。按当前需求，机关固定不转，默认点亮两面的轮廓灯效；`status.motion_enabled=false`、`phase=0`。修改 `mode` 只改变灯效选择，不会使其开始旋转。规则中的匀速/变速运动逻辑仍保留在插件中，但当前场景没有启用。

基地上方的飞镖模块与绿色引导灯整体沿世界 Y 轴在 ±280 mm 内移动（规则第 110–111 页）。支持四种模式：`fixed`、`random_fixed`、`random_moving`、`end_moving`。默认等待开窗事件；`open` 模拟闸门开始开启，7 秒后进入 40 秒检测窗口。随机移动模式在 `fired` 后 600 ms 内换位并静止；末端模式延迟 1.2 秒再于 600 ms 内换位。第一发发出 10 秒后或有效 `hit` 后恢复移动；窗口结束自动回原位。随机位置分布、最小换位距离 80 mm 及 600 ms 内的平滑轨迹是仿真假设，官方未规定这些细节。

飞镖检测模块外观按规则图 4-13 重建：242×239 mm 倒角外壳、140 mm 中部模块、凹入式受击面、两侧护边与灯条、紧固件和下方灯罩。55 mm 绿色透镜采用曲面网格，灯座向下倾斜约48°，正面能看到倾斜透镜的椭圆投影，侧面有厚度与边框。外观重建不代表具备官方传感器精度。

这些道具通过 `/sim/field/*` 接口独立调试，不接入无人机自动冲击或正式比赛裁判协议。能量机关支持 `inactive`、`activating`、`activated` 三种可视状态及五位 `active_mask` 选择亮起的灯臂；尚未模拟弹丸、完整激活判定、流动灯效及全队增益。飞镖 `hit` 是外部测试事件，不替代红外与接触双重检测。机构按运动学约束运动，保留碰撞，不模拟电机力矩。所有运动按仿真时间推进，暂停时冻结，世界重置后清除未执行事件并回位。

地图记录道具的完整运动范围作为避障区域；道具元数据放在 `scene_objects`，`drone_attackable: false`，现有十二块冲击装甲仍是唯一的 `targets`。`/sim/field/energy/status`、`/sim/field/dart_target_red/status`、`/sim/field/dart_target_blue/status` 是含仿真时间的 JSON 状态，仅供调试或离线标注。静态网格中的旧机关和基地顶靶会在生成时按连通部件剔除，避免新旧模型重叠；上游原网格保持完整。地图新增机库 H 标识、队伍色边线和基地启动区轮廓，标线采用当前简化地图尺寸，不能作为官方测量基准。

启动后在另一个已加载 `ws/install/setup.bash` 的终端控制道具，例如：

```bash
# 蓝方基地随机移动靶：开窗后会移动；7秒后可注入发射事件。
ros2 topic pub --once /sim/field/dart_target_blue/mode std_msgs/msg/String "{data: random_moving}"
ros2 topic pub --once /sim/field/dart_target_blue/open std_msgs/msg/Empty '{}'
sleep 7
ros2 topic pub --once /sim/field/dart_target_blue/fired std_msgs/msg/Empty '{}'
# 静止机关的灯效切换；以下命令不会启用旋转。
ros2 topic pub --once /sim/field/energy/mode std_msgs/msg/String "{data: large}"
ros2 topic pub --once /sim/field/energy/blue/state std_msgs/msg/String "{data: activating}"
ros2 topic pub --once /sim/field/energy/blue/active_mask std_msgs/msg/UInt8 '{data: 3}'
```

## 启动仿真

在项目根目录：

```bash
./sim.sh                 # profile=gt：真值只用于验证控制和规划
./sim.sh mono            # 视觉验收：单目 OpenVINS，关闭真值，要求健康估计
./sim.sh stereo          # 视觉验收：双目
./sim.sh off             # ESKF，不启 VIO，也不用真值
./sim.sh gui:=false      # 其余参数原样传给 launch
./sim.sh auto_start:=false # 先查看地图、模型和相机，不自动起飞
```

矛盾组合会在 launch 里直接失败，例如视觉验收再打开 `use_sim_truth:=true`，或在仿真入口使用 `profile:=real`。`use_sim_truth` 不能作为控制输出。模型库路径用仓库根目录，不再写死本机家目录。

`./sim.sh` 会把仿真裁判设成比赛已开始、机库已放行，所以会自己起飞。要停在准备阶段，启动时加 `auto_start:=false`，或把 `/referee/flight_permitted` 和 `/referee/hangar_released` 发成 `false`。

窗口：Gazebo 3D、RViz（轨迹、规划和装甲）、识别调试窗口。

| 参数 | 含义 | launch 默认 |
|---|---|---|
| `team` | `red` / `blue`，机库起飞点与敌方颜色 | `red` |
| `target` | `auto`，或具体装甲如 `blue_outpost_armor_0`、`blue_base_armor_0` | `auto` |
| `profile` | `gt` 真值调试 / `vision` 视觉验收 / `off` 不用视觉也不用真值 | `gt` |
| `estimator` | `gt` / `eskf`；`auto` 跟随 profile | `auto` |
| `vio` | `mono` / `stereo` / `off`；`auto` 跟随 profile | `auto` |
| `run_vio` | 是否启动 OpenVINS；`auto` 跟随 profile | `auto` |
| `use_sim_truth` | 不能当输出来源，显式 true 会启动失败 | `auto` |
| `use_depth` | 是否挂仿真深度相机 | `true` |
| `auto_start` | 阶段和机库都放行后，是否再等几秒自动起飞 | `true` |
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
| RMUC 2026 场地外观网格 | 模型元数据作者为 pb2025 / ITL_Hero_Shoot。相关工程：[北极熊 rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)、[ITL_Hero_Shoot](https://github.com/qwq9966qwq/ITL_Hero_Shoot) | 随上游模型；北极熊仿真器为 Apache-2.0 | 原文件：`ws/src/rm_field_gazebo/models/rmuc_2026`；分区配色衍生网格：`models/rm_field/meshes/field_*` |
| 详细能量机关及科技核心几何 | Blackjack200 / Actor&Thinker，[Daedalus](https://github.com/Blackjack200/bevy_robomaster_simulator)，固定提交 `43681255bc5415cdc15797d54fe7478062af42ed` | AGPL-3.0；原始模型及完整许可一并保留，派生几何不按根目录 MIT 重许可 | `ws/src/rm_field_gazebo/models/rm_field/meshes/source/` 及生成的 `energy_*.stl`（不含自制 `energy_ring.stl`、`energy_plinth.stl`），以及 `technology_core*.stl` |

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
