# This message is part of the IHMC footstep processing toolbox API

# Footstep list containing steps to be mutated
FootstepDataListMessage footstep_data_list_message

# Specifies the position of the initial left stance foot (sole frame) in world frame.
geometry_msgs/Point left_foot_location
# Specifies the orientation of the initial left stance foot (sole frame) in world frame.
geometry_msgs/Quaternion left_foot_orientation

# Specifies the position of the initial right stance foot (sole frame) in world frame.
geometry_msgs/Point right_foot_location
# Specifies the orientation of the initial right stance foot (sole frame) in world frame.
geometry_msgs/Quaternion right_foot_orientation
