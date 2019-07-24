# Phases definition
controller_msgs/StepUpPlannerPhaseParameters[] phases_settings

# Number of control cycles per phase
# Field default value 30
uint64 phase_length

# Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
# Field default value 1
uint64 solver_verbosity

# Maximum distance between the CoM and the foot
# Field default value 2.0
float64 max_leg_length

# Ipopt internal linear solver
string ipopt_linear_solver

# Percentage of the last phase in which weighting the error from the desired value
# Field default value 0.3
float64 final_state_anticipation

# Static friction coefficient
# Field default value 1.0
float64 static_friction_coefficient

# Torsional friction coefficient
# Field default value 1.0
float64 torsional_friction_coefficient

# The weights for the cost function
controller_msgs/StepUpPlannerCostWeights cost_weights

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Include a set of CenterOfMassTrajectoryMessages in the respond
bool include_com_messages

# Send the CenterOfMassTrajectoryMessages directly (the topic name has also to be specified)
bool send_com_messages

# Topic name of IHMC controller accepting the CenterOfMassTrajectoryMessage
string com_messages_topic

# Maximum number of points to be defined in a single CenterOfMassTrajectoryMessage
# Field default value 50
uint64 max_com_message_length

# Include a set of FootstepDataListMessage as well
bool include_footstep_messages

# Send a FootstepDataListMessage as well
bool send_footstep_messages

# Topic name of IHMC controller accepting the FootstepDataListMessage (the topic name has also to be specified)
string footstep_messages_topic


