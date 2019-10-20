# Contains a time stamped position of each vehicle in the swarm, keyed by vehid
# If a vehicle is not a neighbor (so they are not connected) then set to zero.
# The entire message is stamped with the time it was sent, and each position
# is stamped with the time that position was received.

std_msgs/Header header

# An estimate for each vehicle's position in the swarm, keyed by vehicle id.
# If no estimate is available (not connected), send zeros.
geometry_msgs/PointStamped[] positions
