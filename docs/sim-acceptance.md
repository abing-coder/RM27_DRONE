# 单机仿真验收

本轮验证对象为 2027 STEP 场地中的单机视觉导航、装甲接触与返航降落。
`profile:=gt` 是控制/规划对照；`profile:=vision vio:=stereo` 的上层导航不使用仿真真值。
仿真飞控内环仍用真值姿态代表飞控自带姿态估计，尚未替换为完整 PX4 SITL；不能据此声称所有层均无真值。
测试程序订阅真值仅计算误差，不向导航发布评估对齐结果。

## 本轮实际记录（2026-10-26）

以下任务采用双目VIO、默认INT8模型、EGO在线地图，关闭盲打；接触参数已通过实际URDF→SDF转换校验。
记录均来自完整任务，接地误差按停桨前的物理位置计算。

| 任务 | 结果 | 接触法向速度 | 接地水平误差 | 融合位置最大误差 / 原始VIO最大误差 |
|---|---|---|---|---|
| 红方→蓝基地→红机库 | 通过 | 1.162m/s | 2.03cm | 0.291m / 0.349m |
| 蓝方→红基地→蓝机库 | 通过 | 1.108m/s | 4.55cm | 0.180m / 0.258m |
| 红方→蓝前哨站→红机库 | 通过 | 1.297m/s | 0.93cm | 0.894m / 0.824m |

位置误差是真值对照的三维范数，原始VIO仅在启动时作一次评估坐标对齐；融合误差包含固定装甲与H平台校正。
前哨站航段仍出现约0.9m峰值误差，说明全航程精度仍需改进，不能把最后的厘米级接地误差当作全场定位精度。
这些物理接触的冲量低于当前仿真裁判假定的0.15N·s得分门限。
因此`contact_completed`只表示接触任务闭环，不能解释为裁判得分；规则的相关官方阈值仍待定。

## 专项验证

| 验证 | 结果和条件 |
|---|---|
| 模型独立实例验证 | 545张训练图、231张验证图；实例编号2留出。INT8模型在0.7阈值下 TP159、FP2、FN42，关键点平均RMSE1.10px；详见模型 evaluation.json |
| 原生装甲实际接触 | 关闭盲打，YOLO/PnP确认后产生匹配装甲的物理接触事件，未用IMU猜测代替接触；仿真判分阈值仍是假设，不是规则中待定的官方参数 |
| GT完整任务对照 | 曾完成识别、接触、撤离、EGO返航、视觉对中、接地停桨；导航真值误差不作为VIO精度 |
| 实际碰撞参数 | 转换后的碰撞体包含kp2000N/m、kd34.641N·s/m及摩擦系数0.6；脚本逐项核对请求值与SDF值，避免命名导致扩展丢失 |
| 短程GT返航 | 当前物理模型通过起飞、巡航、视觉对中、真实接地与停桨，接地水平误差5.3mm |
| 起飞深度断流 | 当前版本中断2秒，BRAKE后恢复TAKEOFF，再巡航、返航降落，接地水平误差2.9mm |
| 巡航深度断流 | 当前版本实际深度流中继丢弃2秒，通过BRAKE、恢复巡航、返航降落，接地水平误差1.54cm |
| 接触区保护 | 普通动态障碍仍阻挡；新鲜目标接触许可只放行目标小范围；许可过期恢复阻挡，接触区外的近距障碍仍阻挡 |
| 目标区域规划 | 复现精确点被占据的卡点后可生成校验通过的轨迹；允许区域全部被占据时不发布轨迹并报告失败，不清除障碍 |
| 构建与基础检查 | 全工作空间构建、plan_test与safety_test通过；Python/Shell/XML检查及模型、场地哈希核对通过 |
| VIO时间对齐 | 独立ROS测试使用受控时钟、无噪声合成IMU、80ms延迟VIO及1.5rad/s转弯，通过延迟传播与健康检查；不是实机或仿真视觉精度 |

以上是指定条件下的回归记录，不是任意扰动、随机种子或多机运行条件的成功率统计。

## 复现

先使用根目录 `build.sh` 构建，以应用 OpenVINS 静止 ZUPT 启动补丁。
每条测试独立占用一组 ROS domain/Gazebo端口；避免同时复用相同编号。

```bash
source /opt/ros/humble/setup.bash
source ws/install/setup.bash
python3 ws/src/rm_drone_bringup/scripts/sim_acceptance.py \
  --output /tmp/rm27-accept-red --profile vision --team red
python3 ws/src/rm_drone_bringup/scripts/sim_acceptance.py \
  --output /tmp/rm27-accept-blue --profile vision --team blue --domain 157 --port 23519
python3 ws/src/rm_drone_bringup/scripts/sim_acceptance.py \
  --output /tmp/rm27-accept-depth --profile gt --scenario depth-loss --domain 158 --port 23520
python3 ws/src/rm_drone_bringup/scripts/sim_acceptance.py \
  --output /tmp/rm27-accept-takeoff --profile gt --scenario takeoff-depth-loss --domain 158 --port 23520
python3 ws/src/rm_drone_bringup/scripts/sim_acceptance.py \
  --output /tmp/rm27-accept-outpost --profile vision --target blue_outpost_armor_0 --domain 158 --port 23520
```

程序只停止自己启动的进程组，报告包含状态转换、检测帧、匹配目标接触、返航位置、停桨状态、
估计误差与原始VIO误差；图像是实际视觉调试话题。`passed`要求正确目标的真实接触、返航和停桨，
关闭盲打时还要求有效视觉输出；完整任务还要求结果为`contact_completed`，返航测试要求`operator_return`，超时或ESTOP均失败。输出目录是验收结果，检查后可删除。
接地误差记录于仍解锁且收到真实支撑信号时，停桨后的机库夹持归位不会改善该指标。

## 边界

当前在线避障为深度观测后的反应式重规划，不做移动目标速度预测，视场外障碍仍受传感器覆盖限制。
固定装甲地标使用测绘地图中的静态板位置；不等于全局SLAM或任意未知地图重定位。
降落识别依赖原生H平台、颜色、实际尺寸、同步RGB-D及已知机库附近的位置/高度约束，图像失效时不盲降。
尚未完成光照、风扰、相机延迟与丢帧、多机互扰等参数空间的批量统计，也没有板端或实机验收。
世界重置的停桨与旧轨迹取消已在GT模式验证；VIO模式需要重启估计器，不宣称可直接热重置恢复。
