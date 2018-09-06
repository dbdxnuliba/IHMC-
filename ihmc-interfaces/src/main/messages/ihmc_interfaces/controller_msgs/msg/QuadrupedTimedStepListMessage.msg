# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Defines the list of steps to perform
QuadrupedTimedStepMessage[] quadruped_step_list
# If true, the times are expressed in absolute time relative to the robot start
bool is_expressed_in_absolute_time true
# If false, the robot should not shift these step times, and execute immediately
bool can_be_delayed true
# Properties for queueing footstep lists.
QueueableMessage queueing_properties