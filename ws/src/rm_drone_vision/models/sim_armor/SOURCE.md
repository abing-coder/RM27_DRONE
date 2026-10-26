# 仿真装甲四关键点模型与许可

该模型在本项目原生 2027 CAD 场景的实际 Gazebo 渲染图上微调，用于仿真。
初始化权重为 [Ultralytics YOLO11n-pose v8.3.0](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt)，
训练工具为 [Ultralytics v8.3.0](https://github.com/ultralytics/ultralytics/tree/v8.3.0)。
随附 `armor.pt` 训练检查点供后续微调，运行仿真仅需 XML/BIN，不需要 PyTorch。
权重及其派生文件按 **AGPL-3.0** 提供，原文见 [LICENSE.Ultralytics](LICENSE.Ultralytics)；不按仓库 MIT 许可重许可。
CAD、贴纸素材的原有权利与出处仍见场地模型 SOURCE.md 及 docs/credits.md。
旧 RM27 模型保留在上级目录，没有修改原文件，也没有把能量机关五关键点模型改称装甲模型。

输入 `[1,3,640,640]`，RGB float32 /255，居中灰色 114 letterbox；
输出 `[1,16,8400]` = 4 框坐标 + 4 类分数 + 4 对 XY。
类别固定为 `0 red_base / 1 blue_base / 2 red_outpost / 3 blue_outpost`；关键点是灯条 TL、BL、BR、TR。
四类均用小装甲实际灯条尺寸 0.125 m × 0.056 m。默认阈值仍为 0.7，NMS 为 0.3。

渲染脚本 `tools/render_sim_dataset.py` 使用种子 2701 请求 1200 个视角，保留 776 张图；
按装甲实例划分，编号 0、1 用于训练，编号 2 专用于验证，含背景负样本。
可见性由真实渲染深度检查，遮挡/越界目标不作正标注。真值只用于离线标注和评估；
运行时模型只读取图像，不接入地图投影或仿真位姿作为检测结果。
训练为 45 epochs、416 分辨率、batch16、CPU；配方见 `tools/train_sim_armor.py`，参数和哈希见 [training.json](training.json)。

部署版使用 [NNCF](https://github.com/openvinotoolkit/nncf) 2.19.0，MIXED preset，
从训练分组按文件名排序取 256 张图校准为 INT8；验证组没有参与量化校准。
导出/校准流程见 `tools/quantize_sim_armor.py`，精度与耗时见 `evaluation.json`。
这些数字只证明当前仿真域效果，不证明真实相机的泛化性能。
