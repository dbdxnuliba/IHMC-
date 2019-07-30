# Definition of states
int8 STAND=0

int8 SWING=1

# Type of step
int8 state

# Vertices of the left foot for every phase
# (in case of SWING state this is ignored)
controller_msgs/StepUpPlannerVector2[] foot_vertices

# Scale factor to be applied to the foot when planning the trajectories (eventual footsteps will be sent without using such scaling).
# Field default value 1.0
float64 scale


