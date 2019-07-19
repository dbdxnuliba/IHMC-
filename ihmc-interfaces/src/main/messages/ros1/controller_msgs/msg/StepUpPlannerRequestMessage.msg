# Pose of each foot and timings. The number should correspond to the one specified in the StepUpPlannerPhaseParameters message
controller_msgs/StepUpPlannerPhase[] phases

# Position of the CoM to be reached at the end of the final phase
geometry_msgs/Vector3 desired_com_position

# Velocity of the CoM to be reached at the end of the final phase
geometry_msgs/Vector3 desired_com_velocity

# Initial CoM position
geometry_msgs/Vector3 initial_com_position

# Initial CoM velocity
geometry_msgs/Vector3 initial_com_velocity

# Desired value for the final left control
controller_msgs/StepUpPlannerControlElement left_desired_final_control

# Desired value for the final right control
controller_msgs/StepUpPlannerControlElement right_desired_final_control

# Desired leg length
float64 desired_leg_length


