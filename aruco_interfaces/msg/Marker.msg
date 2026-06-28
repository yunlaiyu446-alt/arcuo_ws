# aruco_interfaces/msg/Marker.msg
# Standard header containing timestamp and frame information
std_msgs/Header header

# Unique identifier of the marker
int32 id

# 6x6 binary square marker
# ---0---0---
# -1-0-1-1-0-
# -0-1-1-0-1-
# -1-1-0-1-0-
# -0-0-1-0-1-
# ---1---1---
# This field is intentionally left empty in the message definition
# as the marker data is implicit in the 'id' and its detection.

# 3D pose of the marker relative to the camera frame
# Position (x, y, z) in meters
# Orientation (x, y, z, w) as a quaternion
geometry_msgs/Pose pose