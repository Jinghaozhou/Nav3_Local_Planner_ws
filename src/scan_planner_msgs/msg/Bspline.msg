# Integrated controller contract. Legacy demo publishers may leave these zero.
std_msgs/Header header
uint64 path_id
builtin_interfaces/Time valid_until
int32 order
int64 traj_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt
