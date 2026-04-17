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

  joint_pos_.resize(6, 0.0);
  joint_vel_.resize(6, 0.0);
  joint_torque_.resize(6, 0.0);
  cmd_pos_.resize(6, 0.0);
  cmd_vel_.resize(6, 0.0);
  cmd_torque_.resize(6, 0.0);
  cmd_kp_.resize(6, 0.0);
  cmd_kd_.resize(6, 0.0);

  reset_params();
}

void AirbotPlaySimNode::reset_params()
{
  // reset all the params while preserving the manipulator DOF sizes
  std::fill(joint_pos_.begin(), joint_pos_.end(), 0.0);
  std::fill(joint_vel_.begin(), joint_vel_.end(), 0.0);
  std::fill(joint_torque_.begin(), joint_torque_.end(), 0.0);

  std::fill(cmd_torque_.begin(), cmd_torque_.end(), 0.0);
  std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
  std::fill(cmd_kp_.begin(), cmd_kp_.end(), 0.0);
  std::fill(cmd_kd_.begin(), cmd_kd_.end(), 0.0);
}

void AirbotPlaySimNode::callback_low_state()
{
  // get the sensor data
  if (sim_->d_)
  {
    manipulator_msgs::msg::LowState low_state_msg = manipulator_msgs::msg::LowState();
    low_state_msg.motor_state.resize(joint_pos_.size());

    // lock the thread
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint1_pos")];
    const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint1_vel")];
    const int idx_joint_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint1_tau")];
    for (int i = 0; i < joint_pos_.size(); i++)
    {
      low_state_msg.motor_state[i].q = sim_->d_->sensordata[idx_joint_pos + i];
      joint_pos_[i] = low_state_msg.motor_state[i].q;
      low_state_msg.motor_state[i].dq = sim_->d_->sensordata[idx_joint_vel + i];
      joint_vel_[i] = low_state_msg.motor_state[i].dq;
      low_state_msg.motor_state[i].tau = sim_->d_->sensordata[idx_joint_torque + i];
      joint_torque_[i] = low_state_msg.motor_state[i].tau;
    }

    // publish
    double sim_time = sim_->d_->time;
    low_state_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
    low_state_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg.header.stamp.sec) * 1e9);
    low_state_msg.header.frame_id = "sim_time";

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
        case uint8_t(0):  // no control
          // sim_->d_->ctrl[i] = 0.0;
          break;
        case uint8_t(1):                                                       // position control
          sim_->m_->actuator_gainprm[i * mjNGAIN + 0] = msg->motor_cmd[i].kp;  // set kp
          sim_->m_->actuator_biasprm[i * mjNBIAS + 1] = -msg->motor_cmd[i].kp;
          sim_->m_->actuator_biasprm[i * mjNBIAS + 2] = -msg->motor_cmd[i].kd;  // set kd
          sim_->d_->ctrl[i] = msg->motor_cmd[i].q;
          break;
        case uint8_t(2):  // velocity control
          sim_->m_->actuator_gainprm[(joint_pos_.size() + i) * mjNGAIN + 0] = msg->motor_cmd[i].kd;
          sim_->m_->actuator_biasprm[(joint_pos_.size() + i) * mjNBIAS + 2] = -msg->motor_cmd[i].kd;
          sim_->d_->ctrl[i + joint_pos_.size()] = msg->motor_cmd[i].dq;
          break;
        case uint8_t(3):  // torque control
          sim_->d_->ctrl[i + 2 * joint_pos_.size()] = msg->motor_cmd[i].tau;
          break;
        case uint8_t(4):  // mit mode
          sim_->m_->actuator_gainprm[i * mjNGAIN + 0] = msg->motor_cmd[i].kp;  // set kp
          sim_->m_->actuator_biasprm[i * mjNBIAS + 1] = -msg->motor_cmd[i].kp;
          sim_->m_->actuator_biasprm[i * mjNBIAS + 2] = -msg->motor_cmd[i].kd;  // set kd
          sim_->m_->actuator_gainprm[(joint_pos_.size() + i) * mjNGAIN + 0] = msg->motor_cmd[i].kd;
          sim_->m_->actuator_biasprm[(joint_pos_.size() + i) * mjNBIAS + 2] = 0.0;
          sim_->d_->ctrl[i] = msg->motor_cmd[i].q;
          sim_->d_->ctrl[i + joint_pos_.size()] = msg->motor_cmd[i].dq;
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
      "/airbot_play_gripper/low_cmd", qos,
      std::bind(&AirbotPlayWithGripperSimNode::callback_gripper_low_cmd, this, std::placeholders::_1));

  gripper_state_pub_ptr_ =
      this->create_publisher<end_effector_msgs::msg::GripperState>("/airbot_play_gripper/low_state", qos);

  timers_.emplace_back(
      this->create_wall_timer(2ms, std::bind(&AirbotPlayWithGripperSimNode::callback_gripper_low_state, this)));

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

    AirbotPlaySimNode::callback_low_state();

    // get the data and pub
    const int idx_gripper_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "gripper_joint_pos")];
    const int idx_gripper_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "gripper_joint_vel")];
    const int idx_gripper_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "gripper_joint_tau")];

    gripper_state_msg.gripper_state.q = sim_->d_->sensordata[idx_gripper_pos];
    gripper_pos_[0] = gripper_state_msg.gripper_state.q;
    gripper_state_msg.gripper_state.dq = sim_->d_->sensordata[idx_gripper_vel];
    gripper_vel_[0] = gripper_state_msg.gripper_state.dq;
    gripper_state_msg.gripper_state.tau = sim_->d_->sensordata[idx_gripper_torque];
    gripper_torque_[0] = gripper_state_msg.gripper_state.tau;

    double sim_time = sim_->d_->time;
    gripper_state_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
    gripper_state_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - gripper_state_msg.header.stamp.sec) * 1e9);
    gripper_state_msg.header.frame_id = "sim_time";
    gripper_state_pub_ptr_->publish(gripper_state_msg);
  }
}

void AirbotPlayWithGripperSimNode::callback_gripper_low_cmd(const end_effector_msgs::msg::GripperCmd::SharedPtr msg)
{
  if (sim_->d_)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    const size_t gripper_pos_actuator_idx = 3 * joint_pos_.size();
    const size_t gripper_vel_actuator_idx = gripper_pos_actuator_idx + 1;
    const size_t gripper_tau_actuator_idx = gripper_pos_actuator_idx + 2;

    // apply the gripper motor commands
    switch (msg->gripper_cmd.mode)
    {
      case uint8_t(0):  // no control
        // sim_->d_->ctrl[i] = 0.0;
        break;
      case uint8_t(1):  // position control
        sim_->m_->actuator_gainprm[gripper_pos_actuator_idx * mjNGAIN + 0] = msg->gripper_cmd.kp;  // set kp
        sim_->m_->actuator_biasprm[gripper_pos_actuator_idx * mjNBIAS + 1] = -msg->gripper_cmd.kp;
        sim_->m_->actuator_biasprm[gripper_pos_actuator_idx * mjNBIAS + 2] = -msg->gripper_cmd.kd;  // set kd
        sim_->d_->ctrl[gripper_pos_actuator_idx] = msg->gripper_cmd.q;
        break;
      case uint8_t(2):  // velocity control
        sim_->m_->actuator_gainprm[gripper_vel_actuator_idx * mjNGAIN + 0] = msg->gripper_cmd.kd;
        sim_->m_->actuator_biasprm[gripper_vel_actuator_idx * mjNBIAS + 2] = -msg->gripper_cmd.kd;
        sim_->d_->ctrl[gripper_vel_actuator_idx] = msg->gripper_cmd.dq;
        break;
      case uint8_t(3):  // torque control
        sim_->d_->ctrl[gripper_tau_actuator_idx] = msg->gripper_cmd.tau;
        break;
      case uint8_t(4):  // mit mode
        sim_->m_->actuator_gainprm[gripper_pos_actuator_idx * mjNGAIN + 0] = msg->gripper_cmd.kp;  // set kp
        sim_->m_->actuator_biasprm[gripper_pos_actuator_idx * mjNBIAS + 1] = -msg->gripper_cmd.kp;
        sim_->m_->actuator_biasprm[gripper_pos_actuator_idx * mjNBIAS + 2] = -msg->gripper_cmd.kd;  // set kd
        sim_->m_->actuator_gainprm[gripper_vel_actuator_idx * mjNGAIN + 0] = msg->gripper_cmd.kd;
        sim_->m_->actuator_biasprm[gripper_vel_actuator_idx * mjNBIAS + 2] = 0.0;
        sim_->d_->ctrl[gripper_pos_actuator_idx] = msg->gripper_cmd.q;
        sim_->d_->ctrl[gripper_vel_actuator_idx] = msg->gripper_cmd.dq;
        sim_->d_->ctrl[gripper_tau_actuator_idx] = msg->gripper_cmd.tau;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown control mode: %d for gripper", msg->gripper_cmd.mode);
        break;
    }
  }
}

};  // namespace mujoco_sim
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_sim::AirbotPlaySimNode, mujoco_sim::MujocoSimNodeBase)
PLUGINLIB_EXPORT_CLASS(mujoco_sim::AirbotPlayWithGripperSimNode, mujoco_sim::MujocoSimNodeBase)
