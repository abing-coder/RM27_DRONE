# B-spline in map ENU, knot time zero is header.stamp.
std_msgs/Header header
uint64 request_id
uint64 revision
uint64 map_revision
uint8 degree
float64[] knots
geometry_msgs/Point[] control_points
