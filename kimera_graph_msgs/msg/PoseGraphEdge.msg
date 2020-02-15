std_msgs/Header header

uint64 key_from
uint64 key_to

int32 type

# Type enums
int32 ODOM      = 0
int32 LOOPCLOSE = 1
int32 LANDMARK  = 2
int32 REJECTED_LOOPCLOSE = 3

# Transforms in ede
geometry_msgs/Pose pose
float64[36] covariance
