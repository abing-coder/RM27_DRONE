std_msgs/Header header
bool initialized
bool vio_ok
bool tof_ok
string source           # gt / eskf
float32 position_std    # m
float32 vio_age         # s，距离上一次 VIO 更新
