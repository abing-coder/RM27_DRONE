# 仿真场景与道具

[返回系统介绍](../README.md)

地图采用 Gazebo SDF：`field:=legacy` 使用 `worlds/rmuc2027.world`，`field:=official` 使用 `worlds/rmuc2027_official.world`，均位于 `ws/src/rm_field_gazebo`。无人机本体使用 `ws/src/rm_drone_description/urdf/rm_drone.urdf.xacro`；地图不使用 URDF。

| 后缀 | 用途 |
|---|---|
| `.world` / `.sdf` | 场景、模型、位姿、碰撞与插件定义 |
| `.stl` / `.dae` | 三维几何；带地面纹理的网格使用含 UV 的 DAE |
| `.glb` | 保留的上游详细模型原文件，用于重新转换 |
| `.png` / `.material` | 纹理及颜色、发光等 OGRE 材质 |
| `.yaml` | 规划地图、目标和仿真参数 |
| `.config` | Gazebo 模型名称、版本等元信息 |

以下配色与道具接口说明适用于 `legacy`。

统一灰色场地的版本已保存在提交 `7dadb55`，可用于对比。当前配色参考根目录比赛规则手册印刷第 29–30 页（PDF 第 30–31 页）的图 4-1、4-2、4-3：赛场以深灰地胶、较浅的灰色平台、黑色围挡和设备、红蓝及白色标线为主。木结构表面覆盖 PVC 地胶，不统一涂成棕色木纹。以下是仿真参考色，不是官方 RGB 标准；相机成像还会受到光照与曝光影响。

| 区域 | 参考色 |
|---|---|
| 地胶 | 深石墨灰 `#343638` |
| 高地、公路和平台上表面 | 中性灰 `#7D7D7D` |
| 结构侧面 | 冷灰 `#595B5E` |
| 基地、前哨站等设备外壳 | 石墨灰 `#303438` |
| 围挡 | 近黑 `#191C20` |
| 区域标线 | 白色边界，红蓝队伍色点缀 |

配色定义集中在 `ws/src/rm_field_gazebo/scripts/field_appearance.py`。生成器根据可见面的高度、朝向和设备部件范围拆分原场地网格，保留每个三角形的顶点，给地面和平台补充 UV 及低对比度 PVC 纹理；不再让整张 STL 共用单一灰材质。原 STL 没有材质语义，区域分配仍是近似。装甲图案、灯条材质和绿色引导灯保持独立，调场地颜色不会覆盖它们。

运行 `gen_field.py --skip-background-textures` 也会更新这些配色资源；该开关保留的是原有地面/木纹/围挡背景贴图。原始上游网格和碰撞地图不因配色而改变。

## 新赛季 STEP 地图

用户提供的 `RMUC2027_V1.0.0.stp` 已转换为完整 Gazebo 场景；`能量单元.stp` 是直径约 95 mm、高约 150 mm 的独立能量单元，不是中央能量机关。完整场地中已经有两处补给站、合计十二个单元，不再重复叠放。独立模型 `rmuc2027_energy_unit` 可复用，质量与碰撞为简化值，尚未模拟抓取、磁吸或安装。

```bash
./sim.sh field:=official auto_start:=false
# 原地图
./sim.sh field:=legacy auto_start:=false
```

当前默认仍为 `legacy`，便于对照。新版保留 81 个 CAD 叶实例、40 组唯一几何及面颜色，两个基地、两个前哨站、能量机关、原生飞镖靶、机库与能量单元均来自 STEP。部分复用组件的红蓝面颜色按所属队伍修正。新旧图不混合加载；RViz、imshow、双目、彩色、深度与点云默认配置不变。

装甲保留范围：

| 部位 | 几何及识别范围 |
|---|---|
| 每方基地 | 保留 CAD 的六组装甲结构：三块上部标准小装甲，加三组闭合护甲后的本体装甲；不在护甲外侧伪造额外标准板 |
| 每方前哨站 | 保留三块标准小装甲及原生倾角，固定不转 |
| 标准装甲补充 | 共十二块，沿 CAD 原正面补贴纸、红蓝发光灯条和 1 mm 接触面，原有外壳保留；四角、PnP 尺寸、板面法向与接触面共用坐标 |
| 其他靶面 | CAD 自带的飞镖靶、能量机关及基地本体装甲几何保留；不作为现有四关键点模型的标准装甲导航目标 |

新版保持 CAD 静止姿态，尚未实现基地护甲开闭、飞镖靶移动或能量机关激活判定；下文 `/sim/field/*` 的旧版道具控制不作用于新版。新增十二块标准板配置实际接触插件，但完整冲击任务仍需在新几何上单独验收，不能将模型导入视为赛事得分验证。

坐标以米计，地图 +X 指向蓝方。原 CAD 为毫米、Z 向上，平移场地中心后绕 Z 旋转 180°；原始 STEP 不修改。机库 H 台面实测 1.6 m，生成的 `field_runtime_official.yaml` 同时提供出生点、导航和视觉覆盖参数，并校验世界/地图 SHA-256，避免换了外观却沿用旧目标坐标。当前仿真规划上限为 2.2 m、相对起飞高度 0.5 m、障碍膨胀 0.14 m，这些是适配参数，不是比赛规则数值。

静态物理碰撞使用保留拓扑、按颜色分组简化的 STL；规划使用 CAD 表面生成的 0.15 m 保守体素盒，不使用整块场景的包围盒。ToF 地形查询另用 CAD 向上三角形，保留坡面和分层高度，不用体素顶面冒充台面。规划体素保守覆盖会缩小窄通道；实机仍需实测地图与避障验证。

### 离线重新生成

已经包含生成资源，普通启动不读取 STEP，也不需要安装 CAD 库。需要重建时，在仓库根目录执行，缓存目录放在仓库外：

```bash
python3 -m venv /tmp/rm27-cad-env
/tmp/rm27-cad-env/bin/pip install -r ws/src/rm_field_gazebo/scripts/requirements-cad.txt
/tmp/rm27-cad-env/bin/python ws/src/rm_field_gazebo/scripts/gen_official_field.py \
  --field-step RMUC2027_V1.0.0.stp --energy-step 能量单元.stp \
  --cache-dir /tmp/rm27-cad-cache
```

第一次转换完整装配耗时较长；成功缓存按源文件哈希复用，不重复导入 STEP。修改布局后运行同一命令会同步重建世界、规划地图和启动清单；源 CAD 版本改变必须重新审查 `config/official_layout.yaml` 中的装甲安装面，不能直接套用旧锚点。生成后重新编译 `rm_field_gazebo`、`rm_drone_bringup`。原始文件哈希、转换精度与来源见 [SOURCE.md](../ws/src/rm_field_gazebo/models/rmuc2027_official/SOURCE.md)。

## 重新生成旧版场景

在仓库根目录执行；资源已随仓库提供，正常启动无需重新生成。

```bash
python3 ws/src/rm_field_gazebo/scripts/gen_field.py --skip-background-textures
```

不加该选项时还会重新生成原有背景贴图。

## 装甲与场地道具

装甲正面图案分别取自本地《机器人制作规范手册 V1.0.0（20261015）》印刷第 88–89 页（PDF 第 89–90 页）的附录图 1、3，版权归 DJI；提取为 `rm_armor_outpost.png`、`rm_armor_base.png`，不再使用占位数字。图案仅映射正面并保持原比例，灯条单独使用红蓝自发光材质。基地碰撞/装甲尺寸仍沿用当前 2026 网格的简化模型，贴纸来源不代表场地已经精确复刻 2027 规格。前哨站固定不转。

冲击导引区分装甲板面中心与机身中心，使用地图完整法向及预期末端近水平姿态，补偿简化桨保碰撞盒的接触点。`mission.yaml` 的 `body_radius=0.07`（节点默认）与 `body_half_height=0.0225` 对应当前 URDF 的 140×140×45 mm 盒；更改机身几何时应同步调整。补偿不追随加速时的瞬间俯仰，沿补偿后的板心高度接近，避免目标被不断抬高后先碰到基地上方的飞镖模块。

默认待机距离为 3 m，给加速后的姿态恢复留出距离。末段按 `attack_acceleration`（默认 3 m/s²）加速到 `attack_speed`（默认 3 m/s）；撤离按剩余距离减速回待机点，避免数米位置阶跃造成猛倾斜掉高。再次接近前先稳定位置和速度，`attack_height_tolerance` 默认 8 mm，`attack_vertical_speed_tolerance` 默认 0.03 m/s。这些门限针对当前真值仿真调参，VIO 与实机需按估计噪声重新验证。

任务中的 `attacks` 只统计当前无人机、本次接近、当前目标装甲的实际接触事件；IMU 加速度突变仅触发撤离，穿过板面也不会被当作接触。接触插件按 500 Hz（当前 2 ms 物理步长）采样，入射法向速度使用碰撞响应之前的物理步记录，避免把碰撞后减速误报为入射低速。任务接触次数仍不是裁判得分；力、时长、冲量及速度门限属于模拟裁判假设，不代表官方判定。

能量机关复用 [Daedalus 的 POWER.glb 详细模型](https://github.com/Blackjack200/bevy_robomaster_simulator/blob/43681255bc5415cdc15797d54fe7478062af42ed/assets/POWER.glb)，保留镂空灯臂、外框、分段灯带、靶面、R 标识与支撑结构；下方科技核心与机械臂复用同一项目的 `TECH_CORE.glb`，补建承托支脚的八边形底座。原始 GLB、固定提交号、AGPL v3 许可及转换说明保存在 `ws/src/rm_field_gazebo/models/rm_field/meshes/source/`；生成器会应用节点层级变换、转换坐标轴，再导出 Gazebo Classic 能读取的 STL，不需要另装 Blender 或 Bevy。按当前需求，机关固定不转，默认点亮两面的轮廓灯效；`status.motion_enabled=false`、`phase=0`。修改 `mode` 只改变灯效选择，不会使其开始旋转。规则中的匀速/变速运动逻辑仍保留在插件中，但当前场景没有启用。

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
