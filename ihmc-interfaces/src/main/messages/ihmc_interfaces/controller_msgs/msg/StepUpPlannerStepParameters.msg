# Definition of states
int8 STAND = 0
int8 SWING = 1 

# Type of step
int8 state 0

# Vertices of the left foot for every phase 
# (in case of SWING state this is ignored)
StepUpPlannerVector2[<=10] foot_vertices

# Scale factor to be applied to the foot when planning the trajectories (eventual footsteps will be sent without using such scaling).
float64 scale 1.0