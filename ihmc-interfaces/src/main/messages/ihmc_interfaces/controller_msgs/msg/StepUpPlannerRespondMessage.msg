# The various trajectories are split in every different phase
StepUpPlannerPhaseResult[<=20] phases_result

# The total duration of the generated trajectories
float64 total_duration

# A collection of messages containing the CoM trajectory (0 elements in case not required in the parameters message)
CenterOfMassTrajectoryMessage[<=20] com_messages

# A collection of messages containing the pelvis height trajectory (0 elements in case not required in the parameters message)
PelvisHeightTrajectoryMessage[<=20] pelvis_height_messages

# A collection of messages containing the footsteps (0 elements in case not required in the parameters message)
FootstepDataListMessage[<=20] foostep_messages