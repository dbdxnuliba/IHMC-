# Description of the error
string error_description

# (0) No Error. Using this message to ack a parameter message
# (1) Parameters error
# (2) Parameters not set
# (3) Request error
# (4) Error in the solver
uint8 error_code

# Send back the ID of the received parameters message
uint32 sequence_id_received