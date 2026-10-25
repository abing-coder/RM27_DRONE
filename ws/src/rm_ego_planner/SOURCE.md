# EGO-Planner 来源与集成边界

算法来自 [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)，分支 `ros2_version`，固定提交 `23a8d5a191711dd65633df689bd00f55d4dea8f9`。感谢 Xin Zhou、Jiangchao Zhu、Hongyu Zhou、Chao Xu、Fei Gao 及上游贡献者。本包按 GPL-3.0 分发，完整许可见 LICENSE；带有 Fast-Planner/LGPL 等原始声明的文件继续保留其声明。

引用：Xin Zhou et al., “EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors”, IEEE Robotics and Automation Letters, 2021；“EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments”, ICRA 2021。

`vendor/` 保留上游 B-spline 优化、L-BFGS、局部 A*、多项式初始化、规划管理器与可视化代码。未引入其无人机模拟器、多机网络和轨迹控制器。当前采用单机避障，不启用上游目标运动预测器或集群协同。

上游文件的换行和行尾空白已统一，版权声明保持原文。

适配修改：管理器使用 ROS 节点时钟，支持仿真暂停与重置；可视化标记统一使用 map 坐标系；移除未使用的上游 DataDisp 消息依赖；修复 getMeanVel 缺少返回值并更新 ROS 2 ConstSharedPtr 别名；将局部 A* 工作池限制为 60×60×40，以减少板端分配；修正 A* 初始代价中 `1 >> 20` 的错误；用本项目的 GridMap 查询接口替代上游传感器映射模块。原算法调用的是实际深度融合占据地图，而非仅重命名原有 minimum-snap。

本项目新增的 PointCloud2 映射层按图像时间查询 TF，使用逐帧 hit/miss 证据与射线清除；已知场地可以作为静态先验，比赛围栏与禁区始终保留。移动物体通过观测更新触发重规划，不包含目标速度估计或未来轨迹预测。未观测到的障碍不会按计时自动消失。

点云读取支持有组织和无组织数据，每帧射线数量有硬上限。关闭巡航几何先验时，任务层与映射层使用同一套起飞区域保留规则。

任务节点是唯一的飞机设定点发布者。EGO 仅提供带请求 ID 和地图版本的 B-spline；任务节点在接收、执行及终端导引阶段保留定位、深度健康与刹停保护。实现和验收细节见仓库 docs/navigation.md。
