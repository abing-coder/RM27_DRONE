# 视觉识别与 YOLO11

[返回系统介绍](../README.md)

## 数据流

```text
/drone/attack/image_raw（基地）或 color/image_raw（前哨站）+ camera_info
  → BGR8 → 等比缩放 / 对应模型的 letterbox → RGB / 255
  → YOLO11 → 敌方普通装甲类别筛选 → 关键点排序与 NMS
  → IPPE PnP → 距离与重投影门限
  → 相机位姿 + 图像时刻的机体位姿 → /drone/armors
```

传统后端以灯条配对替代 YOLO；两者共用物理尺寸、PnP、位姿同步和下游任务接口。没有识别结果时，YOLO 后端发布空数组，不静默切换传统后端。

## 当前仿真默认模型

默认使用 `models/sim_armor/armor.xml/.bin`：Ultralytics YOLO11n-pose 在实际 Gazebo
渲染图上微调并经 NNCF INT8 量化，四类与四个灯条端点，输入 640×640、输出 `[1,16,8400]`。
预处理采用居中灰色 114 letterbox。旧 RM27 模型仍支持原有 `[1,50,8400]`、38 类和右下补黑，
两种格式明确分支解码，不混用类别表或预处理。训练来源、数据划分与许可见
[模型说明](../ws/src/rm_drone_vision/models/sim_armor/SOURCE.md)。

基地任务默认读取 `/drone/attack/image_raw` 和内参，对应固定下倾0.49rad、水平视场1.0rad的
斜视相机；显式前哨站任务读取原来的`/drone/color/image_raw`。原有前视彩色、左右目、深度图、
点云与RViz显示全部保留。基地板朝上，观察点在板外水平1.5m、上方约0.8m，不能沿用旧图的
水平3m观察点。PnP使用被选择相机的实际内外参，`pose_map`保留三维板面姿态，局部+X为朝外法向。

独立装甲实例验证集上，0.7 门限与 IoU0.5 下精确率约 98.8%、召回率约 79.1%；
关键点平均 RMSE1.10 px。i9-14900HX、OpenVINO CPU2线程的 INT8 推理中位约17.5 ms、P95约20 ms。
该数字来自独立图像评估，整套系统的实时帧率受渲染、定位、规划等影响；不作为板端或实机精度。

## 依赖与构建

默认构建要求 OpenVINO C++ Runtime。遵循 [OpenVINO 安装说明](https://docs.openvino.ai/2026/get-started/install-openvino.html) 安装适合主机架构的 Runtime，并加载环境。例如安装目录为 `/opt/intel/openvino_2026.4.1` 时：

```bash
source /opt/intel/openvino_2026.4.1/setupvars.sh
source /opt/ros/humble/setup.bash
./build.sh --packages-up-to rm_drone_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

若安装方式不含 `setupvars.sh`，用实际安装目录设置 `OpenVINO_DIR` 指向含有 `OpenVINOConfig.cmake` 的目录，并确保运行时动态库可被加载。仅安装 Python 包不保证提供本项目需要的 C++ CMake 配置。

仅构建传统后端时：

```bash
colcon build --symlink-install --packages-select rm_drone_vision \
  --cmake-args -DRM_ENABLE_OPENVINO=OFF
# 在仓库根目录启动
./sim.sh detector:=traditional auto_start:=false
```

重新启用 YOLO 需要把该 CMake 选项设回 `ON` 并重新构建。未构建 OpenVINO 后端却选择 YOLO 时，节点会明确报错退出。

模型加载失败或识别节点异常退出时，启动器会关闭本次仿真，避免其他节点继续运行而误以为识别仍在线。

## 旧 RM27 对照模型与输出格式

项目包含用户提供的 `yolo11.xml` 和 `yolo11.bin` 的副本，安装到 `rm_drone_vision` 的 `share/models` 下，运行时不依赖原 RM27 目录。来源、SHA-256、原工程许可见 [models/SOURCE.md](../ws/src/rm_drone_vision/models/SOURCE.md)。

这是**装甲板专用 YOLO 四关键点模型（YOLO-keypoint）**：网络同时输出框、类别和四个二维关键点。它不是只输出框的普通检测模型，也不是通用人体姿态模型；没有逐点的可见性/置信度通道，不能套用通用人体 pose 解码器。核对依据是 RM27 的 `tasks/auto_aim/yolos/yolo11.hpp` 中 `class_num_=38` 及 `.cpp` 中通道 `42..49` 的四对 XY 解码；本项目模型副本与来源文件哈希一致。XML 中的 `model_type=YOLO` 是通用标签，不能单凭它判断是否包含关键点。

模型已经是 OpenVINO IR，`.xml` 描述网络，`.bin` 保存权重，仿真通过 OpenVINO C++ Runtime 在 CPU 上推理，不需要再导出模型：

| 项目 | 定义 |
|---|---|
| 输入 | `[1,3,640,640]`，RGB、float32、除以 255 |
| 图像缩放 | 保持比例，左上对齐，右侧/底部补黑；坐标还原除以同一缩放系数 |
| 输出 | `[1,50,8400]` |
| 通道 | `cx,cy,w,h` + 38 个类别分数 + 4 对 XY 关键点 |
| PnP 顺序 | 左上、左下、右下、右上，表示两根灯条端点 |
| 后处理 | 敌方颜色/类别过滤、几何合法性检查、按类别 NMS |

解码器支持以下类别，任务按当前地图的装甲类型关联：

| 类别 | 蓝方 ID | 红方 ID | PnP 类型 |
|---|---|---|---|
| 前哨站普通装甲 | 18 | 19 | 小装甲，当前 STEP 图 125 mm |
| 基地大装甲（兼容解码，当前图无此目标） | 21 | 22 | 大装甲，灯条中心间距 242 mm |
| 基地小装甲 | 25 | 26 | 小装甲，灯条中心间距 125 mm |

灯条长度均为 56 mm。新图灯条覆盖件中心间距按 CAD 测量约 124.4 mm，仿真发光面统一取 125 mm，由场地清单同时驱动生成几何和 PnP 参数。小基地类 `25/26` 按小装甲求解，并按类别单独 NMS；熄灭、紫色及其他机器人类别不会进入无人机冲击流程。类别 ID 不等同于 `field_map_official.yaml` 中供离线标注的四类 `armor_classes=0..3`。

旧RM27模型在运行时只加载，不训练、不转换。YOLOv5、YOLOv8、能量机关模型及 `best2-sim.onnx` 具有不同输出布局，不能直接传给本后端；程序会检查输入/输出形状并拒绝不匹配模型。

## 配置与运行

推理参数配置：[vision.yaml](../ws/src/rm_drone_vision/config/vision.yaml)。调整后重启节点生效；不支持运行中热切换模型。

| 节点参数 | 默认值 | 含义 |
|---|---|---|
| `detector_backend` | `yolo`，独立节点与 launch 一致 | `yolo` 或 `traditional` |
| `yolo_model` | launch 自动定位已安装模型 | XML 路径，同名 BIN 放在同目录 |
| `yolo_device` | `CPU` | OpenVINO 设备 |
| `yolo_score_threshold` | `0.7` | 类别分数门限 |
| `yolo_nms_threshold` | `0.3` | NMS IoU 门限 |
| `yolo_threads` | `2` | 直接使用 CPU 设备时的推理线程数 |
| `show_features` | `true` | 原分辨率 FAST，最多 250 个绿色角点，仅用于预览；可通过同名 launch 参数关闭 |
| `show_yolo_rejected` | `true` | 无有效候选时显示分数至少 0.3 的最高分被拒绝预测框，橙色虚线，仅用于预览 |
| `show_yolo_bbox` | `false` | 额外显示网络外接矩形；不影响检测、PnP及导航 |
| `max_reprojection_error` | `4.0` px | 四个关键点平均重投影误差上限 |
| `pose_sync_slop` | `0.05` s | 位姿插值区间最大时长 |

从仓库根目录启动：

```bash
./sim.sh detector:=yolo auto_start:=false
./sim.sh detector:=traditional auto_start:=false
./sim.sh detector:=yolo yolo_model:=/absolute/path/yolo11.xml auto_start:=false
```

检查识别图像和输出：

```bash
source /opt/ros/humble/setup.bash
source ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /drone/vision/debug_image
ros2 topic echo /drone/armors
```

调试图像还显示原始最高分类 `model best=cID score` 与过门限候选数 `raw`，可定位低置信度或类别不匹配。调试图像显示后端、候选数、通过 PnP 的数量、推理耗时、类别分数、关键点和距离。框存在但 `poses=0` 时，应检查关键点、物理尺寸和 PnP 门限。完全无候选时，应先检查原图、敌方颜色、视距和模型域差异。

imshow 与 `/drone/vision/debug_image` 共用预览。收到过期图像时仍运行 YOLO，显示检测框/关键点/耗时，并标注橙色 `PREVIEW ONLY ... navigation disabled`；此帧不发布装甲定位。缺少内参时仍可推理，显示 `Waiting for camera_info`；有 PnP 但缺少同步 odom 时显示 `camera pose only`。窗口一直 `Waiting for camera` 表示尚未收到可转换的图像，应先检查 `/drone/color/image_raw`；窗口底部的 `camera ... fps / received / input gap` 使用墙钟统计输入；`input gap` 持续增大说明相机没有继续出图，和模型没有检出目标是两个问题。顶部 `yolo/OpenVINO #N ... ms` 的帧号递增表示网络确实在运行，`candidates=0` 表示过滤后无有效候选，不代表未推理。

绿色圆点及`FAST corners ... (preview)`是角点预览，不是YOLO或VIO检测结果。默认每个YOLO有效候选只画四点轮廓：黄色表示二维关键点，PnP合格后同一轮廓改为绿色。四点是灯条端点，构成PnP使用的几何边界；网络的外接矩形是另一种表示，`show_yolo_bbox:=true`才额外显示，因此同一装甲不再默认叠两个框。多个实际装甲仍各有轮廓。

无有效候选时，原始预测分数至少0.3且框合法才显示橙色虚线`PREVIEW ... rejected`，只解释类别、分数或关键点拒绝原因，不进入PnP或任务；定位门限仍为0.7。关闭诊断可用`show_features:=false show_yolo_rejected:=false`。

## 时间与任务接口

推理在专用线程执行，只保留一帧待处理图像，较新的图像覆盖尚未处理的旧图像，避免积压阻塞 ROS 回调。模型编译和推理请求在启动时建立，逐帧复用，不保存调试图片。

相机消息按 `encoding` 转成 BGR8。最多缓存三帧等待位姿，最多等待 100 ms；插值使用图像采样时刻前后的 odom。相机内参和该时刻的位姿在送入推理线程时做快照。图像超过 200 ms、时间回退或跨世界重置的推理结果不发布到 `/drone/armors`，预览仍显示图像与原因。订阅队列深度为 1，不追赶历史图像。没有匹配 odom 时，可发布相机系位姿，但 `has_map_pose=false`，任务节点不会将其作为地图目标。

PnP 输出位于 `/drone/armors` 的每个 `ArmorDetection` 中：`pose_camera` 是相机光学系下的装甲位姿（米，+z 向前）；`pose_map` 仅在 `has_map_pose=true` 时有效，其位置保留三维坐标，姿态保留完整板面倾角，局部 +X 表示朝外法向；任务同时校验地图目标的法向与位置。`distance` 是相机至装甲中心的距离，`reprojection_error` 是像素重投影误差。求解用的是两根灯条的四个端点与配置的实际尺寸，而非二维检测框；图像四点可以估计板面位姿，但不等同于无人机应飞到的机身中心位置。

`ArmorDetection` 保留原有几何接口，并增加：

| 字段 | YOLO | 传统灯条 |
|---|---|---|
| `kind` | `base` / `outpost` | 空字符串，类别未知 |
| `confidence` | 原始类别分数 | `0`，不冒充类别置信度 |
| `class_id` | 模型原始 ID | `-1` |
| `has_depth_normal` | 同步深度验证板面法向时为true | 同样需深度验证；类别未知时不参与固定装甲航向校正 |

开启深度导引时，装甲内部区域的三维点可验证板面法向：至少10点，平面内两个方向均非退化，点到平面的RMS不超过2mm，并与PnP方向及中心深度一致。成功后重新校验四点重投影误差，才标记`has_depth_normal`。未通过时保留原PnP结果，不把不可靠法向用于绝对航向约束。此几何筛选按当前仿真深度设置，实机噪声需另行标定。

任务节点继续检查敌方颜色、装甲大小和已知目标位置，YOLO 还需类别匹配。二维框四角不会直接输入 PnP，场景几何真值不作为识别输出。

验证识别驱动的冲击时加 `allow_blind_attack:=false`。此开关与 VIO 配置独立；即使用真值定位，也可以单独验证视觉目标确认：

```bash
./sim.sh gt detector:=yolo allow_blind_attack:=false
```

视觉定位默认锁步，GT默认异步，可用 `lockstep:=true/false` 显式选择。IMU在物理步结束时同步采样并使用该物理时间戳；双目20Hz、物理/IMU500Hz。图像与点云的时效检查始终保留，不通过放宽门限接收旧数据。

相机渲染通过 `render_backend:=auto` 自动选择：`glxinfo -B` 检测到直接硬件 OpenGL 时使用桌面渲染（本机为 Intel D3D12），否则使用 Xvfb 软件渲染。安装 `mesa-utils` 提供检测工具；可显式设置 `render_backend:=native` 或 `software` 排查兼容性。Gazebo 渲染后端与 OpenVINO 推理设备独立，硬件渲染不代表 YOLO 改用 GPU。场景加载和首帧初始化仍需数秒，窗口会显示等待的相机话题。

Gazebo 窗口使用 `gui_render_backend:=auto/native/software` 单独选择渲染，`auto` 跟随相机后端。启动时输出其 DISPLAY 与渲染方式；窗口通过 Qt XCB 显示，关闭 X11 共享内存传图以兼容 WSLg。出现有窗口但黑屏时可先用 `./sim.sh gui_render_backend:=software auto_start:=false`，相机仍可保持硬件渲染；软件渲染的场景窗口帧率可能明显降低，仅作为兼容选项。`gui:=false` 只启动服务端，没有 Gazebo 场景窗口；imshow 是独立窗口。

所有仿真模式均保留左右目图像 `/drone/stereo/left/*`、`/drone/stereo/right/*`，不再按 VIO 模式裁剪可视化数据源；`mono` 的估计器只使用左目，`stereo` 使用双目，`gt/off` 不启动 OpenVINS。彩色识别相机保持 640×480、20 Hz 的仿真采样配置；深度相机默认开启，发布 `/drone/depth/depth/image_raw` 和 `/drone/depth/points`，供原有 RViz 深度图和点云显示使用。只有显式传入 `use_depth:=false` 才关闭深度数据源，下视 ToF 不受影响。仿真看门狗与数据按同一仿真时钟计龄，锁步等待不被算成实机断流；实机的单调时钟保护不受影响。

训练或评估使用对应的原始相机图像，不使用带检测框的debug图像。`tools/render_sim_dataset.py`、`train_sim_armor.py` 与 `quantize_sim_armor.py` 提供场景采集、关键点标注、微调和量化流程；配方与独立实例评估见默认模型的SOURCE.md和evaluation.json。旧权重保留作对照，不能把新模型的评估结果归给旧权重。

## 旧权重的历史对照结果（2026-10-22）

本机 OpenVINO CPU 两线程抽样运行本地真实录像，暖机后单帧推理约 16–23 ms；这是主机上的模型调用耗时，不是机载平台性能承诺，也不是正式精度指标。

对旧版简化场景红蓝基地、前哨站的四个固定近景，旧权重最高候选常为熄灭前哨站/熄灭小基地（ID 20/27），分数约 0.31–0.58，默认 0.7 门限下无有效目标。这反映了仿真外观与旧训练域的差异，不能通过把类别强改为敌方或任意降低阈值解决。默认启动现在直接运行 OpenVINO YOLO，并在预览和终端显示实际推理帧号与耗时。过滤阈值保持不变；传统灯条只通过 `detector:=traditional` 显式选择，不做静默回退。

后续适配应从原始相机采集覆盖距离、俯仰、背景和曝光的样本，按灯条端点标注关键点，建立独立验证集再微调。替换权重时保持输出约定，或同步实现对应解码器；用 `allow_blind_attack:=false` 验证视觉确认，分别统计漏检、误检及 PnP 位置误差。

2026-10-22 新 STEP 场地验证：红蓝两侧均经过真值模式起飞、规划到巡航，双目、彩色、深度、点云及预览有数据，点云光学系可转换到 `map`。四个标准装甲正视近景均实际运行同一 C++ OpenVINO 后端：最高分约 0.12–0.47，默认 0.7 门限下均无有效检测，部分前哨站仍被预测为小基地。地图/类别接入完成不代表旧权重已经适应新图；后续需要采集带四关键点标注的多视角、距离与光照数据，按场景划分训练/验证集后微调并重新验收。不要用预览框冒充有效识别，或降低门限掩盖类别错误。


## 深度辅助装甲距离

仿真启用深度时，YOLO仍负责类别、框和四关键点，IPPE PnP仍计算板面位姿。
识别线程匹配图像时间附近30毫秒内的深度点云，按相机之间的实际外参投影，
对装甲中心附近点取中位深度，并检查数量、深度离散和重投影误差。
距离校正使用真实深度观测，避免小装甲关键点像素误差被放大为数十厘米的距离误差。
没有合格的同步深度时保留相机系预览，拒绝其地图导航输出；不能用深度独立生成装甲检测。
两相机的内外参和时间同步必须对应实机标定，不能直接沿用仿真数值。
