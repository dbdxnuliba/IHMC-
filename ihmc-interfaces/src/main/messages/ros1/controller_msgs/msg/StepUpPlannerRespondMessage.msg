# The various trajectories are split in every different phase
controller_msgs/StepUpPlannerPhaseResult[] phases_result

# A collection of messages containing the CoM trajectory (0 elements in case not required in the parameters message)
controller_msgs/CenterOfMassTrajectoryMessage[] com_messages

# A collection of messages containing the footsteps (0 elements in case not required in the parameters message)
controller_msgs/FootstepDataListMessage[] foostep_messages


