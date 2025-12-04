std_msgs/Header header

sensor_msgs/Imu imu
common_msgs/MotorState[] motor_state # Dynamic size for different robot morphologies

# common_msgs/MotorState[3] head_motor_state
# common_msgs/MotorState[7] left_arm_motor_state
# common_msgs/MotorState[7] right_arm_motor_state
# common_msgs/MotorState[3] waist_motor_state
# common_msgs/MotorState[6] left_leg_motor_state
# common_msgs/MotorState[6] right_leg_motor_state