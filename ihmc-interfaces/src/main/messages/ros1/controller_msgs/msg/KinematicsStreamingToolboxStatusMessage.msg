int8 KSTSTATE_SLEEP=0

int8 KSTSTATE_CALIBRATION=1

int8 KSTSTATE_VALIDATION=2

int8 KSTSTATE_STREAMING=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Indicates the current state of the toolbox:
# Field default value 255
int8 current_state


