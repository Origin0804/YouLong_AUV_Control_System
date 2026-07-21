# Pose information published by basic_motion
builtin_interfaces/Time stamp

# odom origin in map frame (degrees for yaw)
float32 origin_x
float32 origin_y
float32 origin_z
float32 origin_yaw

# robot current pose in odom frame (degrees for yaw)
float32 robot_x
float32 robot_y
float32 robot_z
float32 robot_roll    # Roll in degrees
float32 robot_pitch   # Pitch in degrees
float32 robot_yaw

# current motion target in odom frame (degrees for yaw)
float32 target_x
float32 target_y
float32 target_z
float32 target_yaw
