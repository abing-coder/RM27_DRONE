# 姿态+推力设定值，对应 PX4 VehicleAttitudeSetpoint / VehicleRatesSetpoint
# 坐标约定：map 为 ENU，机体为 FLU（PX4 桥内部转换为 NED/FRD）
uint8 MODE_ATTITUDE = 0
uint8 MODE_RATES = 1

std_msgs/Header header
uint8 mode
geometry_msgs/Quaternion orientation   # map -> body
geometry_msgs/Vector3 body_rate        # rad/s，MODE_RATES 时使用，MODE_ATTITUDE 时作为前馈
float32 thrust                         # 归一化总推力 [0,1]
bool armed
