# This message is part of the KinematicsStreamingToolbox API.
# This message can be used to configure the toolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Request a re-calibration. If the toolbox is currently streaming or validating, then it stops and falls back to the calibration state.
bool request_calibration false