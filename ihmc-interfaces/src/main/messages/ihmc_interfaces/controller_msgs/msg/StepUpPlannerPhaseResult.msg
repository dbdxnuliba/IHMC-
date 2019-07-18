# Message containing the result of the Step-Up planner for a single phase. The dT is equal to the duration divided by the number of element in the vectors.

# CoM position trajectory of this phase
geometry_msgs/Vector3[<=200] com_position

# CoM velocity trajectory of this phase
geometry_msgs/Vector3[<=200] com_velocity

# CoM acceleration trajectory of this phase
geometry_msgs/Vector3[<=200] com_acceleration

# Left control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
StepUpPlannerControlElement[<=200] left_controls

# Right control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
StepUpPlannerControlElement[<=200] right_controls

# Duration of the phase
float64 duration