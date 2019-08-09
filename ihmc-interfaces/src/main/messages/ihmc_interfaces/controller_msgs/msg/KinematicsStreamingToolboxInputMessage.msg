# This message is part of the KinematicsStreamingToolbox API.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool control_center_of_mass false
KinematicsToolboxCenterOfMassMessage center_of_mass_input
KinematicsToolboxRigidBodyMessage[<=20] rigid_body_inputs