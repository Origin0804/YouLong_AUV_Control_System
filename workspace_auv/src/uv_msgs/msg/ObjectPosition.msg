# World position of a tracked object

uint16 class_id
uint32 instance_id    # instance index within same class_id (0, 1, 2, ...)
string class_name
float32 world_x       # NED North (m)
float32 world_y       # NED East (m)
float32 world_z       # Depth (m)
float32 confidence    # Position estimate confidence
uint32 num_observations
