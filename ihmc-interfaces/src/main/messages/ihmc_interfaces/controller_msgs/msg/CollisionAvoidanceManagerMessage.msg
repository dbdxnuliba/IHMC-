
byte MODE_OVERRIDE = 0
byte MODE_ADD = 1

# List of the planar regions considered for collision avoidance.
PlanarRegionMessage[<=20] planar_regions_list

# Considers only the edges for collision avoidance
bool consider_only_edges false

# When equal to MODE_OVERRIDE, regions previously sent are cleared and only these regions will be considered. 
# When equal to MODE_ADD, the regions sent with this message are added. The "consider_only_edges" parameter will affect only the regions sent within this message
byte mode 0

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id