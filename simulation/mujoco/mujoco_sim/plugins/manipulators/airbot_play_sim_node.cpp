#include "manipulators/airbot_play_sim_node.hpp"

namespace mujoco_sim
{

AirbotPlaySimNode::AirbotPlaySimNode() : MujocoSimNodeBase("airbot_play_sim")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  cmd_sub_ptr_ = this->create_subscription<manipulator_msgs::msg::LowCmd>(
      "/airbot_play/low_cmd", qos, std::bind(&AirbotPlaySimNode::callback_low_cmd, this, std::placeholders::_1));

  low_state_pub_ptr_ = this->create_publisher<manipulator_msgs::msg::LowState>("/airbot_play/low_state", qos);

  timers_.emplace_back(this->create_wall_timer(2ms, std::bind(&AirbotPlaySimNode::callback_low_state, this)));

  reset_params();
}

void AirbotPlaySimNode::reset_params()
{
  // reset all the params
  // sensor readings
  joint_pos_.clear();
  joint_vel_.clear();
  joint_torque_.clear();

  // motor commands
  cmd_torque_.clear();
  cmd_pos_.clear();
  cmd_vel_.clear();
  cmd_kp_.clear();
  cmd_kd_.clear();
}

void AirbotPlaySimNode::callback_low_state()
{
  // get the sensor data
  if (sim_->d_)
  {
    manipulator_msgs::msg::LowState low_state_msg = manipulator_msgs::msg::LowState();

    // lock the thread
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    // TODO: get the data

    // publish
    double sim_time = sim_->d_->time;
    low_state_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
    low_state_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg.header.stamp.sec) * 1e9);
    low_state_msg.header.frame_id = "sim_time";
    // low_state_msg.joint_position = joint_pos_;
    // low_state_msg.joint_velocity = joint_vel_;
    // low_state_msg.joint_torque = joint_torque_;

    low_state_pub_ptr_->publish(low_state_msg);
  }

}

void AirbotPlaySimNode::callback_low_cmd(const manipulator_msgs::msg::LowCmd::SharedPtr msg)
{
  if (sim_->d_)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    // apply the motor commands
    for (size_t i = 0; i < joint_pos_.size(); i++)
    {
      switch (msg->motor_cmd[i].mode)
      {
      case uint8_t(0): // no control
        // sim_->d_->ctrl[i] = 0.0;
        break;
      case uint8_t(1): // position control
        sim_->m_->actuator_gainprm[i * mjNGAIN + 0] = msg->motor_cmd[i].kp; // set kp
        sim_->m_->actuator_biasprm[i * mjNBIAS + 1] = -msg->motor_cmd[i].kp;
        sim_->m_->actuator_biasprm[i * mjNBIAS + 2] = -msg->motor_cmd[i].kd; // set kd
        sim_->d_->ctrl[i] = msg->motor_cmd[i].q;
        break;
      case uint8_t(2): // velocity control
        sim_->m_->actuator_gainprm[(joint_pos_.size() + i) * mjNGAIN + 0] = msg->motor_cmd[i].kd;
        sim_->m_->actuator_biasprm[(joint_pos_.size() + i) * mjNBIAS + 2] = -msg->motor_cmd[i].kd;
        sim_->d_->ctrl[i + joint_pos_.size()] = msg->motor_cmd[i].dq;
        break;
      case uint8_t(3): // torque control
        sim_->d_->ctrl[i + 2 * joint_pos_.size()] = msg->motor_cmd[i].tau;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown control mode: %d for joint %zu", msg->motor_cmd[i].mode, i);
        break;
      }
    }
  }
}

AirbotPlayWithGripperSimNode::AirbotPlayWithGripperSimNode() : AirbotPlaySimNode()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  gripper_cmd_sub_ptr_ = this->create_subscription<end_effector_msgs::msg::GripperCmd>(
      "/airbot_play_gripper/low_cmd", qos, std::bind(&AirbotPlayWithGripperSimNode::callback_gripper_low_cmd, this, std::placeholders::_1));

  gripper_state_pub_ptr_ = this->create_publisher<end_effector_msgs::msg::GripperState>("/airbot_play_gripper/low_state", qos);

  timers_.emplace_back(this->create_wall_timer(2ms, std::bind(&AirbotPlayWithGripperSimNode::callback_gripper_low_state, this)));

  reset_params();

}

void AirbotPlayWithGripperSimNode::reset_params()
{
  AirbotPlaySimNode::reset_params();

  // reset gripper params
  gripper_pos_[0] = 0.0;
  gripper_vel_[0] = 0.0;
  gripper_torque_[0] = 0.0;

  gripper_mode_[0] = 0.0;
  gripper_cmd_pos_[0] = 0.0;
  gripper_cmd_vel_[0] = 0.0;
  gripper_cmd_torque_[0] = 0.0;
}

void AirbotPlayWithGripperSimNode::callback_gripper_low_state()
{
  // get the gripper sensor data
  if (sim_->d_)
  {
    end_effector_msgs::msg::GripperState gripper_state_msg = end_effector_msgs::msg::GripperState();

    // lock the thread
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    // TODO: get the data

    // TODO: pub the data
  }
}

void AirbotPlayWithGripperSimNode::callback_gripper_low_cmd(const end_effector_msgs::msg::GripperCmd::SharedPtr msg)
{
  if (sim_->d_)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    // apply the gripper motor commands
    switch (msg->gripper_cmd.mode)
    {
    case uint8_t(0): // no control
      // sim_->d_->ctrl[i] = 0.0;
      break;
    case uint8_t(1): // position control
      sim_->m_->actuator_gainprm[0 * mjNGAIN + 0] = msg->gripper_cmd.kp; // set kp
      sim_->m_->actuator_biasprm[0 * mjNBIAS + 1] = -msg->gripper_cmd.kp;
      sim_->m_->actuator_biasprm[0 * mjNBIAS + 2] = -msg->gripper_cmd.kd; // set kd
      sim_->d_->ctrl[0] = msg->gripper_cmd.q;
      break;
    case uint8_t(2): // velocity control
      sim_->m_->actuator_gainprm[(0 + joint_pos_.size()) * mjNGAIN + 0] = msg->gripper_cmd.kd;
      sim_->m_->actuator_biasprm[(0 + joint_pos_.size()) * mjNBIAS + 2] = -msg->gripper_cmd.kd;
      sim_->d_->ctrl[0 + joint_pos_.size()] = msg->gripper_cmd.dq;
      break;
    case uint8_t(3): // torque control
      sim_->d_->ctrl[0 + 2 * joint_pos_.size()] = msg->gripper_cmd.tau;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown control mode: %d for gripper", msg->gripper_cmd.mode);
      break;
    }
  }
}

}; // namespace mujoco_sim
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_sim::AirbotPlaySimNode, mujoco_sim::MujocoSimNodeBase)
PLUGINLIB_EXPORT_CLASS(mujoco_sim::AirbotPlayWithGripperSimNode, mujoco_sim::MujocoSimNodeBase)
