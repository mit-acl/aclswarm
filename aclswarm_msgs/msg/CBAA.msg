Header header

# A list of agent's local price for each task
float32[] price

# An index list about who set the highest price for each task. -1 if not set.
int32[] who

# The current bid iteration of the current CBAA auction
uint32 iter
