Header header

# The iteration this bid is associated with
uint32 iter

# This agent's cost table, i.e., its price for each task
float32[] price

# An index list about who set the highest price for each task. -1 if not set.
int32[] who
