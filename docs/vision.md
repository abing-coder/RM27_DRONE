# 视觉识别与 YOLO11

[返回系统介绍](../README.md)

## 数据流

```text
/drone/color/image_raw + camera_info
  → BGR8 → 等比缩放 / 右下补黑 → RGB / 255
  → YOLO11 → 敌方普通装甲类别筛选 → 关键点排序与 NMS
  → IPPE PnP → 距离与重投影门限
  → 相机位姿 + 图像时刻的机体位姿 → /drone/armors
```

传统后端以灯条配对替代 YOLO；两者共用物理尺寸、PnP、位姿同步和下游任务接口。没有识别结果时，YOLO 后端发布空数组，不静默切换传统后端。

## 依赖与构建

默认构建要求 OpenVINO C++ Runtime。遵循 [OpenVINO 安装说明](https://docs.openvino.ai/2026/get-started/install-openvino.html) 安装适合主机架构的 Runtime，并加载环境。例如安装目录为 `/opt/intel/openvino_2026.4.1` 时：

```bash
source /opt/intel/openvino_2026.4.1/setupvars.sh
source /opt/ros/humble/setup.bash
cd ws
colcon build --symlink-install --packages-up-to rm_drone_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

若安装方式不含 `setupvars.sh`，用实际安装目录设置 `OpenVINO_DIR` 指向含有 `OpenVINOConfig.cmake` 的目录，并确保运行时动态库可被加载。仅安装 Python 包不保证提供本项目需要的 C++ CMake 配置。

仅构建传统后端时：

```bash
colcon build --symlink-install --packages-select rm_drone_vision \
  --cmake-args -DRM_ENABLE_OPENVINO=OFF
# 回到仓库根目录启动
./sim.sh detector:=traditional auto_start:=false
```

重新启用 YOLO 需要把该 CMake 选项设回 `ON` 并重新构建。未构建 OpenVINO 后端却选择 YOLO 时，节点会明确报错退出。

模型加载失败或识别节点异常退出时，启动器会关闭本次仿真，避免其他节点继续运行而误以为识别仍在线。

## 模型与输出格式

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

当前场景只接受以下类别：

| 类别 | 蓝方 ID | 红方 ID | PnP 类型 |
|---|---|---|---|
| 前哨站普通装甲 | 18 | 19 | 小装甲，灯条中心间距 147 mm |
| 基地大装甲 | 21 | 22 | 大装甲，灯条中心间距 242 mm |

灯条长度均为 56 mm，遵循当前场景几何。小基地类 `25/26` 不会被强行当作大基地；熄灭、紫色及其他机器人类别不会进入无人机冲击流程。类别 ID 不等同于 `field_map.yaml` 中供离线标注的四类 `armor_classes=0..3`。

模型文件只做加载，不训练、不转换。YOLOv5、YOLOv8、能量机关模型及 `best2-sim.onnx` 具有不同输出布局，不能直接传给本后端；程序会检查输入/输出形状并拒绝不匹配模型。

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
| `max_reprojection_error` | `4.0` px | 四个关键点平均重投影误差上限 |
| `pose_sync_slop` | `0.05` s | 位姿插值区间最大时长 |

从仓库根目录启动：

```bash
./sim.sh detector:=yolo lockstep:=true auto_start:=false
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

## 时间与任务接口

推理在专用线程执行，只保留一帧待处理图像，较新的图像覆盖尚未处理的旧图像，避免积压阻塞 ROS 回调。模型编译和推理请求在启动时建立，逐帧复用，不保存调试图片。

相机消息按 `encoding` 转成 BGR8。最多缓存三帧等待位姿，最多等待 100 ms；插值使用图像采样时刻前后的 odom。相机内参和该时刻的位姿在送入推理线程时做快照。图像超过 200 ms、时间回退或跨世界重置的推理结果不发布到 `/drone/armors`，预览仍显示图像与原因。订阅队列深度为 1，不追赶历史图像。没有匹配 odom 时，可发布相机系位姿，但 `has_map_pose=false`，任务节点不会将其作为地图目标。

PnP 输出位于 `/drone/armors` 的每个 `ArmorDetection` 中：`pose_camera` 是相机光学系下的装甲位姿（米，+z 向前）；`pose_map` 仅在 `has_map_pose=true` 时有效，其位置保留三维坐标，姿态当前只表示朝外法向的水平偏航；冲击几何的完整倾角来自场地地图。`distance` 是相机至装甲中心的距离，`reprojection_error` 是像素重投影误差。求解用的是两根灯条的四个端点与配置的实际尺寸，而非二维检测框；图像四点可以估计板面位姿，但不等同于无人机应飞到的机身中心位置。

`ArmorDetection` 保留原有几何接口，并增加：

| 字段 | YOLO | 传统灯条 |
|---|---|---|
| `kind` | `base` / `outpost` | 空字符串，类别未知 |
| `confidence` | 原始类别分数 | `0`，不冒充类别置信度 |
| `class_id` | 模型原始 ID | `-1` |

任务节点继续检查敌方颜色、装甲大小和已知目标位置，YOLO 还需类别匹配。二维框四角不会直接输入 PnP，场景几何真值不作为识别输出。

验证识别驱动的冲击时加 `allow_blind_attack:=false`。此开关与 VIO 配置独立；即使用真值定位，也可以单独验证视觉目标确认：

```bash
./sim.sh gt detector:=yolo allow_blind_attack:=false
```

仿真入口的 `lockstep:=auto` 在启用 VIO 或 `allow_blind_attack:=false` 时开启 Gazebo `--lockstep`，让物理推进等待相机渲染，避免软件渲染下图像落后时钟数秒。真值控制调试且允许盲冲击时默认不锁步，避免渲染阻塞起飞。此模式下过期图像仅供预览，不能当作视觉闭环验收；检查识别可显式设置 `lockstep:=true`。锁步模式可能降低实时因子，不通过放宽图像时效门限接受旧数据。

相机渲染通过 `render_backend:=auto` 自动选择：`glxinfo -B` 检测到直接硬件 OpenGL 时使用桌面渲染（本机为 Intel D3D12），否则使用 Xvfb 软件渲染。安装 `mesa-utils` 提供检测工具；可显式设置 `render_backend:=native` 或 `software` 排查兼容性。Gazebo 渲染后端与 OpenVINO 推理设备独立，硬件渲染不代表 YOLO 改用 GPU。场景加载和首帧初始化仍需数秒，窗口会显示等待的相机话题。

只生成当前 VIO 模式需要的仿真相机：`gt/off` 不渲染额外 VIO 相机，`mono` 只渲染左目并沿用 `/drone/stereo/left/*`，`stereo` 渲染双目。彩色识别相机保持 640×480、15 Hz 的仿真采样配置；可选深度相机默认关闭，用 `use_depth:=true` 显式开启，下视 ToF 不受影响。仿真看门狗与数据按同一仿真时钟计龄，锁步等待不被算成实机断流；实机的单调时钟保护不受影响。

训练或评估应使用 `/drone/color/image_raw`，不使用带检测框的 debug 图像。当前不含批量采集、自动标注、训练或正式精度评测；本地旧权重对 2027 装甲贴图和 Gazebo 外观的适应性需要单独评估。

## 当前权重验证结果

本机 OpenVINO CPU 两线程抽样运行本地真实录像，暖机后单帧推理约 16–23 ms；这是主机上的模型调用耗时，不是机载平台性能承诺，也不是正式精度指标。

对当前场景红蓝基地、前哨站的四个固定近景，旧权重最高候选常为熄灭前哨站/熄灭小基地（ID 20/27），分数约 0.31–0.58，默认 0.7 门限下无有效目标。这反映了仿真外观与旧训练域的差异，不能通过把类别强改为敌方或任意降低阈值解决。默认启动现在直接运行 OpenVINO YOLO，并在预览和终端显示实际推理帧号与耗时。类别映射和阈值保持不变；传统灯条只通过 `detector:=traditional` 显式选择，不做静默回退。

后续适配应从原始相机采集覆盖距离、俯仰、背景和曝光的样本，按灯条端点标注关键点，建立独立验证集再微调。替换权重时保持输出约定，或同步实现对应解码器；用 `allow_blind_attack:=false` 验证视觉确认，分别统计漏检、误检及 PnP 位置误差。
