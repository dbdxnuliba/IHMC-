# This message is part of the IHMC whole-body inverse kinematics module.
# This output status will be converted into the WholeBodyTrajectoryMessage.
# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of times for each key frames.
# The length of this should be same with the length of the configurations. 
float64[] key_frame_times

# List of configurations for each key frames.
controller_msgs/KinematicsToolboxOutputStatus[] robot_configurations

# Solution quality.
# The total summation of the all solution quality for each key frames. 
float64 solution_quality -1