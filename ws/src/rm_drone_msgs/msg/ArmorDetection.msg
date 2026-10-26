uint8 SMALL = 0
uint8 LARGE = 1

uint8 armor_type
string color                    # red / blue
string kind                     # base / outpost；传统灯条后端为空（类别未知）
float32 confidence              # YOLO 类别置信度；传统灯条后端为 0，不代表概率
int32 class_id                  # YOLO11 原始类别 ID；传统灯条后端为 -1
geometry_msgs/Pose pose_camera  # 相机光学坐标系下装甲位姿
geometry_msgs/Pose pose_map     # map 坐标系下装甲位姿（估计器可用时）
bool has_map_pose
bool has_depth_normal           # 法向经过同步深度平面验证；可用于固定装甲航向约束
float32 distance
float32 reprojection_error
float32[8] corners              # 图像角点 (x,y)*4：左上、左下、右下、右上
