# This message is part of the KinematicsStreamingToolbox API.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool use_ground_height false
bool use_head_pose false
bool use_left_hand_pose false
bool use_right_hand_pose false

float64 ground_height
geometry_msgs/Pose head_pose
geometry_msgs/Pose left_hand_pose
geometry_msgs/Pose right_hand_pose