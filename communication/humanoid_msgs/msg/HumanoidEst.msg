std_msgs/Header header

# legs
common_msgs/MotorState[] motor_state
common_msgs/ContactSensor[2] contact_state # contact indicator
common_msgs/FTSensor[2] contact_wrench # contact wrench

# base
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel  