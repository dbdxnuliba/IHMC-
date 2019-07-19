# CoM position trajectory of this phase
geometry_msgs/Vector3[] com_position

# CoM velocity trajectory of this phase
geometry_msgs/Vector3[] com_velocity

# CoM acceleration trajectory of this phase
geometry_msgs/Vector3[] com_acceleration

# Left control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
controller_msgs/StepUpPlannerControlElement[] left_controls

# Right control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
controller_msgs/StepUpPlannerControlElement[] right_controls

# Duration of the phase
float64 duration


