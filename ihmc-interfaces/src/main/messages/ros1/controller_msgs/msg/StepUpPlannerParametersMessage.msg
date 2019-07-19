# Phases definition
controller_msgs/StepUpPlannerPhaseParameters[] phases_settings

# Number of control cycles per phase
uint64 phase_length

# Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
uint8 solver_verbosity

# Maximum distance between the CoM and the foot
float64 max_leg_length

# Ipopt internal linear solver
string ipopt_linear_solver

# Percentage of the last phase in which weighting the error from the desired value
float64 final_state_anticipation

# Static friction coefficient
float64 static_friction_coefficient

# Torsional friction coefficient
float64 torsional_friction_coefficient

# The weights for the cost function
controller_msgs/StepUpPlannerCostWeights cost_weights

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id


