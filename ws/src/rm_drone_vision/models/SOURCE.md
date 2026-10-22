# YOLO11 装甲关键点模型

这两个文件由用户授权从本机 `/home/abing/RM27/assets` 复制，原目录只读。
程序运行时只使用本仓库/安装目录中的副本。

本地工程记录的 origin 为 [abing-coder/RM27](https://github.com/abing-coder/RM27)，
核对时的 HEAD 为 `08be1a05967ab163ef76994b7842ade0cd550525`。本地副本可能含未提交修改，
这里的提交号仅记录核对环境；复制模型的确切内容以以下 SHA-256 为准。
相关视觉代码保留 TongjiSuperPower 版权，其开源视觉框架见
[TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)。
不据此推断当前权重最初由哪个团队训练。

| 文件 | SHA-256 |
|---|---|
| `yolo11.xml` | `c513dea404aa704d7e4923441b4b51a0f1c3ee54186f5215a04048c297a239c0` |
| `yolo11.bin` | `ace99cf0ce36a7ac5deb0d779ef0a15e952a67da9f57f6e16119e093d713963c` |

推理约定参考本地工程的 `tasks/auto_aim/yolos/yolo11.cpp/.hpp`，
类别对应参考 `tasks/auto_aim/armor.hpp`。其根目录 MIT 许可版权为
`Copyright (c) 2025 TongjiSuperPower`，完整原文保存在
[LICENSE.TongjiSuperPower](LICENSE.TongjiSuperPower)。本项目重新实现 ROS 2 接口和推理封装，未复制该工程的调试图片保存逻辑。
模型训练数据、精度和权重独立授权未在本地模型文件中提供，随附代码许可不作为这些信息的证明。

- 输入：`[1,3,640,640]`，RGB float32 / 255。
- 预处理：等比缩放置于左上角，右侧和底部补黑；不是居中灰色 letterbox。
- 输出：`[1,50,8400]`，通道为 `cx,cy,w,h`、38 个类别分数、4 对 XY 关键点。这是装甲四关键点检测网络，不是纯框检测或标准人体姿态网络。
- XML 的通用元数据标有 `model_type=YOLO` 和 `pad_value=114`；实际预处理以核对的 RM27 推理实现为准，沿用右下补黑，不据此改成居中灰色填充。
- 当前无人机仅接受敌方普通装甲：蓝/红前哨站 `18/19`，蓝/红大基地 `21/22`、小基地 `25/26`。
- 小基地 `25/26` 使用小装甲 PnP；当前 STEP 图为小基地，保留大装甲类别解码供兼容使用。任务仍按地图的 kind/armor_type/位置关联，不把小装甲强改成大装甲。
- 关键点统一成左上、左下、右下、右上，然后进入现有 IPPE PnP。
- 这些 ID 与规划地图的四类离线标注 `0..3` **不是同一套类别表**。

同目录其他 YOLOv5、YOLOv8、buff 模型及 `best2-sim.onnx` 输出格式不同，
没有作为此后端的可互换模型。替换模型时必须一起验证类别表、关键点定义与预处理。
