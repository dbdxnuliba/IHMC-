# The state of a robot expressed in the Cartesian frame
# global time along trajectory
controller_msgs/Duration time_from_start

# Position, velocity and acceleration of the base expressed in world frame
# The orientation quaternion maps base to world frame.
# base pos/vel/acc in world
controller_msgs/State6d base

# endeffector pos/vel/acc in world
controller_msgs/StateLin3d[] ee_motion

# endeffector forces expressed in world
geometry_msgs/Vector3[] ee_forces

# True if the foot is touching the environment
bool[] ee_contact


