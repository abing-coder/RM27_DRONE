std_msgs/Header header
bool initialized
bool vio_ok
bool tof_ok
string source           # ground_truth / disabled / waiting / degraded / healthy / imu_gap
float32 position_std    # m
float32 vio_age         # s，距离上一次 VIO 更新
