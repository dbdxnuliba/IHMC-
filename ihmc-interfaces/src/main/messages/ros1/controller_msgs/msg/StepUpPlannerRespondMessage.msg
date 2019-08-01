# The various trajectories are split in every different phase
controller_msgs/StepUpPlannerPhaseResult[] phases_result

# The total duration of the generated trajectories
float64 total_duration

# A collection of messages containing the CoM trajectory (0 elements in case not required in the parameters message)
controller_msgs/CenterOfMassTrajectoryMessage[] com_messages

# A collection of messages containing the pelvis height trajectory (0 elements in case not required in the parameters message)
controller_msgs/PelvisHeightTrajectoryMessage[] pelvis_height_messages

# A collection of messages containing the footsteps (0 elements in case not required in the parameters message)
controller_msgs/FootstepDataListMessage[] foostep_messages


