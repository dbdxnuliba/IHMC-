# Should disappear for the ROS equivalent.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 timestamp
geometry_msgs/Point sensor_position
geometry_msgs/Quaternion sensor_orientation
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 angular_velocity
float32[<=600000] point_cloud