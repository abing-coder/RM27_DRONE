# 能量单元来源

几何来自用户提供的 `能量单元.stp`，不是中央能量机关模型。

- 原始 STEP SHA-256：`998c5b394dde349311359afb3a547d1ce64296c30c8e45b0aefb1d092860eca9`。
- 文件头记录：Creo Parametric，2026-09-22；下载地址未随文件提供。
- 原始 CAD 留在项目根目录，不加入 Git；转换后的几何保留其来源，不能仅按本项目 MIT 许可重新归属。
- 转换：Open CASCADE 7.8.1；毫米转米，CAD Y-up 转 ENU Z-up，保留面颜色。
- 质量按规则手册第 4.5.1 节取 0.4 kg；惯量及外包络圆柱碰撞为仿真近似，未实现磁吸、抓取或装配判定。
- 生成入口：`scripts/step_xde.py` 与 `scripts/gen_official_field.py`。
