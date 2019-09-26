Header header

# A list of agent's local price for each task
float32[] price

# An index list about who set the highest price for each task. -1 if not set
int16[] who

# Iteration number of the CBAA algorithm
uint16 cbaa_iter
