# Mission owns the request ID. Inactive requests cancel local navigation.
std_msgs/Header header
uint64 request_id
geometry_msgs/Point position
# Radius of the acceptable free endpoint region, metres; zero means exact.
float64 tolerance
bool active
