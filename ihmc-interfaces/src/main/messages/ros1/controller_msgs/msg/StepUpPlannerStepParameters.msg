# Definition of states
int8 STAND=0

int8 SWING=1

# Type of step
int8 state

# Vertices of the left foot for every phase
# (in case of SWING state this is ignored)
controller_msgs/StepUpPlannerVector2[] foot_vertices


