# Phases definition
StepUpPlannerPhaseParameters[<=20] phases_settings

# Number of control cycles per phase
uint64 phase_length 30

# Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
uint64 solver_verbosity 1

# Maximum distance between the CoM and the foot
float64 max_leg_length 2.0

# Ipopt internal linear solver
string ipopt_linear_solver

# Percentage of the last phase in which weighting the error from the desired value
float64 final_state_anticipation 0.3

# Static friction coefficient
float64 static_friction_coefficient 1.0

# Torsional friction coefficient
float64 torsional_friction_coefficient 1.0

# The weights for the cost function
StepUpPlannerCostWeights cost_weights

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Include a set of CenterOfMassTrajectoryMessages in the respond
bool include_com_messages false

# Send the CenterOfMassTrajectoryMessages directly (the topic name has also to be specified)
bool send_com_messages false

# Topic name of IHMC controller accepting the CenterOfMassTrajectoryMessage
string com_messages_topic

# Maximum number of points to be defined in a single CenterOfMassTrajectoryMessage
uint64 max_com_message_length 50

# Include a set of FootstepDataListMessage as well
bool include_footstep_messages false

# Send a FootstepDataListMessage as well
bool send_footstep_messages false

# Topic name of IHMC controller accepting the FootstepDataListMessage (the topic name has also to be specified)
string footstep_messages_topic
