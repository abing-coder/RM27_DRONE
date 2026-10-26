# 仿真图片说明

这些图片用于项目 README，图片路径均为仓库内的相对路径。

| 文件 | 内容与来源 |
|---|---|
| `field-overview.png` | 2026-10-22 在 Gazebo 中实际渲染的新赛季完整场地 |
| `armor-red-base.png` | 同次场景渲染的红方基地标准小装甲近景 |
| `armor-blue-outpost.png` | 同次场景渲染的蓝方前哨站标准小装甲近景 |
| `simulation-rviz.png` | 2026-10-22 运行当前新地图，任务进入 `CRUISE` 后截取的 RViz 窗口 |
| `simulation-vision.png` | 同次运行中订阅 `/drone/vision/debug_image` 保存的实际图像 |

场地与装甲近景来自当前 `rmuc2027_official.world`。RViz 与视觉预览采集使用 `profile:=gt auto_start:=true render_backend:=software`，地图采用默认新赛季场地；采集时未改动仓库内 RViz 配置。截图展示的是仿真真值调试，不能作为视觉惯性定位或实机性能的验收记录。

视觉预览中的绿色点为 FAST 角点。该帧没有有效装甲候选，且标记 `PREVIEW ONLY`，不作为成功识别或视觉导航的示例。

几何来自用户提供的 RMUC 2027 STEP 文件，出处及哈希见[CAD 来源说明](../../ws/src/rm_field_gazebo/models/rmuc2027_official/SOURCE.md)。装甲图案来自 DJI《机器人制作规范手册》，具体版本和页码见[场景文档](../simulation.md#装甲与场地道具)。第三方几何、图案和官方标识保留原权利人权利，截图不改变其许可归属。


`sim-detection-red-base.jpg`、`sim-detection-blue-outpost.jpg`：2026-10-25 的真实 Gazebo 渲染独立验证视角，
采用当前 INT8 YOLO 模型输出的检测框与四关键点，不使用真值框代替预测。文字中的 c0/c3 对应
新模型的红基地/蓝前哨站类别；来源、模型许可与数据划分见 sim_armor/SOURCE.md。
