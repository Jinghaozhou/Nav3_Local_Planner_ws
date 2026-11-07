uint8 WAITING_INPUT=0
uint8 WAITING_PATH=1
uint8 RUNNING=2
uint8 GOAL_REACHED=3
uint8 LOCAL_PLAN_FAILED=4
uint8 GLOBAL_PATH_INVALID=5
uint8 LOCALIZATION_UNAVAILABLE=6
uint8 DATA_STALE=7
uint8 TF_CHANGED=8

std_msgs/Header header
uint64 path_id
int64 traj_id
uint8 state
bool motion_allowed
string reason
# Original task goal, expressed in header.frame_id (normally map).
geometry_msgs/Point goal
