# Phases definition
StepUpPlannerPhaseParameters[<=20] phases_settings

# Number of control cycles per phase
uint64 phase_length 30

# Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
uint8 solver_verbosity 1

# Maximum distance between the CoM and the foot
float64 max_leg_length 2.0

# Ipopt internal linear solver
string ipopt_linear_solver mumps

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

# Send a CenterOfMassTrajectoryMessage as well
bool send_com_message false

# Topic name of IHMC controller accepting the CenterOfMassTrajectoryMessage
string com_message_topic

# Maximum number of points to be defined in a single CenterOfMassTrajectoryMessage
uint64 max_com_message_length 50

# Send a FootstepDataListMessage as well
bool send_footstep_message false

# Topic name of IHMC controller accepting the FootstepDataListMessage
string footstep_message_topic