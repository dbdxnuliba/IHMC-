# This message is part of the IHMC quadruped controller API.
# This message sends a desired stepping velocity to the step teleop module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Defines if the step list is adjustable
bool are_steps_adjustable true

# Desired planar velocities in m/s and rad/s. X and Y correspond to forward and lateral velocities. Z corresponds to yaw velocity
geometry_msgs/Vector3 desired_velocity

# Desired xgait settings
QuadrupedXGaitSettingsPacket x_gait_settings