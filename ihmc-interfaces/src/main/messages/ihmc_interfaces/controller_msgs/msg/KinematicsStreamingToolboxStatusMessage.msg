# This message is part of the IHMC whole-body inverse kinematics module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte KSTSTATE_SLEEP = 0
byte KSTSTATE_CALIBRATION = 1
byte KSTSTATE_VALIDATION = 2
byte KSTSTATE_STREAMING = 3

# Indicates the current state of the toolbox:
byte current_state 255