# A formation is defined by a graph with vertices located at 3D points
# and edges that represent which formation points need to obtain
# relative position information (via communication). The graph is encoded
# with an adjacency matrix.

std_msgs/Header header

# formation name
string name

# 3D points of the formation
geometry_msgs/Point[] points

# Adjacency matrix encoding which formation points needs to communicate
std_msgs/UInt8MultiArray adjmat

# (optional/debug) The operator may send pre-calculated gains
std_msgs/Float32MultiArray gains
