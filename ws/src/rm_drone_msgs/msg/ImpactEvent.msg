# 装甲模块“冲击事件”（规则表 5-1）：接触力超过 alpha N 开始，全部低于 alpha N 结束
std_msgs/Header header
string armor_name
string striker_name
float64 duration         # 接触时间 s
float64 impulse          # 累积冲量 N*s（仿真中以 ∫F dt 近似 ∬F dt dS）
float64 peak_force       # N
float64 approach_speed   # 撞击前沿装甲法向的接近速度 m/s
