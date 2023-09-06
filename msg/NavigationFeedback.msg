Header header
string robot_name
# string request_id

uint8 status
uint8 IDLE=2
uint8 RUNNING=3
uint8 FINISHED=4
uint8 CANCELLED=5

float64 progress
geometry_msgs/PoseStamped base_position