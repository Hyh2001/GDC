#include "humanoids/tron1_sim_node.hpp"

namespace mujoco_sim
{

Tron1SimNode::Tron1SimNode() : MujocoSimNodeBase("tron1_sim")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  cmd_sub_ptr_ = this->create_subscription<humanoid_msgs::msg::LowCmd>(
      "/tron1/low_cmd", qos, std::bind(&Tron1SimNode::callback_low_cmd, this, std::placeholders::_1));

  low_state_pub_ptr_ = this->create_publisher<humanoid_msgs::msg::LowState>("/tron1/low_state", qos);

  timers_.emplace_back(this->create_wall_timer(2ms, std::bind(&Tron1SimNode::callback_low_state, this)));

  // Initialize vectors with default size (8 joints for wheel_foot)
  joint_pos_.resize(8, 0.0f);
  joint_vel_.resize(8, 0.0f);
  joint_torque_.resize(8, 0.0f);
  mode_.resize(8, 0);  // default mode 0
  cmd_torque_.resize(8, 0.0f);
  cmd_pos_.resize(8, 0.0f);
  cmd_vel_.resize(8, 0.0f);
  cmd_kp_.resize(8, 0.0f);
  cmd_kd_.resize(8, 0.0f);
  low_state_msg_.motor_state.resize(8);

  reset_params();
}

void Tron1SimNode::reset_params()
{
  // reset all the params
  // sensor readings
  std::fill(std::begin(joint_pos_), std::end(joint_pos_), 0.0);
  std::fill(std::begin(joint_vel_), std::end(joint_vel_), 0.0);
  std::fill(std::begin(gyro_), std::end(gyro_), 0.0);
  std::fill(std::begin(accelerom_), std::end(accelerom_), 0.0);
  std::fill(std::begin(contact_), std::end(contact_), false);

  // motor commands
  std::fill(std::begin(mode_), std::end(mode_), 0);
  std::fill(std::begin(cmd_torque_), std::end(cmd_torque_), 0.0);
  std::fill(std::begin(cmd_pos_), std::end(cmd_pos_), 0.0);
  std::fill(std::begin(cmd_vel_), std::end(cmd_vel_), 0.0);
  // motor params
  std::fill(std::begin(cmd_kp_), std::end(cmd_kp_), 0.0);
  std::fill(std::begin(cmd_kd_), std::end(cmd_kd_), 0.0);
}

void Tron1SimNode::load_gripper_gains_from_xml()
{
  gripper_kp_from_xml_ = 0.0f;
  gripper_kd_from_xml_ = 0.0f;
  gripper_gains_from_xml_loaded_ = false;

  if (!sim_ || !sim_->m_)
  {
    RCLCPP_WARN(this->get_logger(), "MuJoCo model is not loaded; gripper XML gains are unavailable.");
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

  const int actuator_id = mj_name2id(sim_->m_, mjOBJ_ACTUATOR, "gripper_joint_pos");
  if (actuator_id < 0)
  {
    RCLCPP_WARN(this->get_logger(), "Could not find MuJoCo actuator 'gripper_joint_pos'; using gripper command kp/kd.");
    return;
  }

  gripper_kp_from_xml_ = static_cast<float>(sim_->m_->actuator_gainprm[actuator_id * mjNGAIN + 0]);
  gripper_kd_from_xml_ = static_cast<float>(-sim_->m_->actuator_biasprm[actuator_id * mjNBIAS + 2]);
  gripper_gains_from_xml_loaded_ = true;

  if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM && cmd_kp_.size() > 12 && cmd_kd_.size() > 12)
  {
    cmd_kp_[12] = gripper_kp_from_xml_;
    cmd_kd_[12] = gripper_kd_from_xml_;
  }
  else if ((robot_type_ == RobotType::FLAT_FOOT_WITH_ARM || robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM) &&
           cmd_kp_.size() > 14 && cmd_kd_.size() > 14)
  {
    cmd_kp_[14] = gripper_kp_from_xml_;
    cmd_kd_[14] = gripper_kd_from_xml_;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded gripper gains from XML: kp=%f kd=%f",
              gripper_kp_from_xml_, gripper_kd_from_xml_);
}

void Tron1SimNode::load_ros2_params()
{
  // load robot type
  std::string robot_type_str;
  this->declare_parameter<std::string>("robot_type", "point_foot");
  robot_type_str = this->get_parameter("robot_type").as_string();
  robot_type_ = parse_robot_type(robot_type_str);

  // if type is point foot, low the q related number to 6
  if (robot_type_ == RobotType::POINT_FOOT)
  {
    std::cout << "Loading point foot robot type." << std::endl;
    joint_pos_.resize(6, 0.0f);
    joint_vel_.resize(6, 0.0f);
    joint_torque_.resize(6, 0.0f);
    mode_.resize(6, 0);
    cmd_torque_.resize(6, 0.0f);
    cmd_pos_.resize(6, 0.0f);
    cmd_vel_.resize(6, 0.0f);
    cmd_kp_.resize(6, 0.0f);
    cmd_kd_.resize(6, 0.0f);
    low_state_msg_.motor_state.resize(6);
  }
  else if(robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    std::cout << "Loading point foot with arm." << std::endl;
    joint_pos_.resize(13, 0.0f);
    joint_vel_.resize(13, 0.0f);
    joint_torque_.resize(13, 0.0f);
    mode_.resize(13, 0);
    cmd_torque_.resize(13, 0.0f);
    cmd_pos_.resize(13, 0.0f);
    cmd_vel_.resize(13, 0.0f);
    cmd_kp_.resize(13, 0.0f);
    cmd_kd_.resize(13, 0.0f);
    low_state_msg_.motor_state.resize(6);
    // initialize sub and pub
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    manipulator_cmd_sub_ptr_ = this->create_subscription<manipulator_msgs::msg::LowCmd>(
      "/airbot_play/low_cmd", qos, std::bind(&Tron1SimNode::callback_manipulator_low_cmd, this, std::placeholders::_1));
    manipulator_state_pub_ptr_ = this->create_publisher<manipulator_msgs::msg::LowState>("/airbot_play/low_state", qos);

    gripper_cmd_sub_ptr_ = this->create_subscription<end_effector_msgs::msg::GripperCmd>(
      "/airbot_play_gripper/gripper_cmd", qos, std::bind(&Tron1SimNode::callback_gripper_low_cmd, this, std::placeholders::_1));
    gripper_state_pub_ptr_ = this->create_publisher<end_effector_msgs::msg::GripperState>("/airbot_play_gripper/gripper_state", qos);
    // resize manipulator msg
    manipulator_low_state_msg_.motor_state.resize(6);
  }
  // if type is with arm then add 6 to joint related number
  else if (robot_type_ == RobotType::FLAT_FOOT_WITH_ARM ||
           robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
  {
    std::cout << "Loading flat foot or wheel foot with arm." << std::endl;
    joint_pos_.resize(15, 0.0f);
    joint_vel_.resize(15, 0.0f);
    joint_torque_.resize(15, 0.0f);
    mode_.resize(15, 0);
    cmd_torque_.resize(15, 0.0f);
    cmd_pos_.resize(15, 0.0f);
    cmd_vel_.resize(15, 0.0f);
    cmd_kp_.resize(15, 0.0f);
    cmd_kd_.resize(15, 0.0f);
    low_state_msg_.motor_state.resize(8);
    // initialize sub and pub
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    manipulator_cmd_sub_ptr_ = this->create_subscription<manipulator_msgs::msg::LowCmd>(
      "/airbot_play/low_cmd", qos, std::bind(&Tron1SimNode::callback_manipulator_low_cmd, this, std::placeholders::_1));
    manipulator_state_pub_ptr_ = this->create_publisher<manipulator_msgs::msg::LowState>("/airbot_play/low_state", qos);

    gripper_cmd_sub_ptr_ = this->create_subscription<end_effector_msgs::msg::GripperCmd>(
      "/airbot_play_gripper/gripper_cmd", qos, std::bind(&Tron1SimNode::callback_gripper_low_cmd, this, std::placeholders::_1));
    gripper_state_pub_ptr_ = this->create_publisher<end_effector_msgs::msg::GripperState>("/airbot_play_gripper/gripper_state", qos);
    // resize manipulator msg
    manipulator_low_state_msg_.motor_state.resize(6);
  }
  if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM ||
      robot_type_ == RobotType::FLAT_FOOT_WITH_ARM ||
      robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
  {
    load_gripper_gains_from_xml();
  }

  RCLCPP_INFO(this->get_logger(), "Robot type: %s is loaded (enum value: %d)", robot_type_str.c_str(),
              static_cast<int>(robot_type_));
}

Tron1SimNode::RobotType Tron1SimNode::parse_robot_type(const std::string& robot_type_str)
{
  if (robot_type_str == "point_foot" || robot_type_str == "POINT_FOOT")
  {
    return RobotType::POINT_FOOT;
  }
  else if (robot_type_str == "flat_foot" || robot_type_str == "FLAT_FOOT")
  {
    return RobotType::FLAT_FOOT;
  }
  else if (robot_type_str == "wheel_foot" || robot_type_str == "WHEEL_FOOT")
  {
    return RobotType::WHEEL_FOOT;
  }
  else if (robot_type_str == "point_foot_with_arm" || robot_type_str == "POINT_FOOT_WITH_ARM")
  {
    return RobotType::POINT_FOOT_WITH_ARM;
  }
  else if (robot_type_str == "flat_foot_with_arm" || robot_type_str == "FLAT_FOOT_WITH_ARM")
  {
    return RobotType::FLAT_FOOT_WITH_ARM;
  }
  else if (robot_type_str == "wheel_foot_with_arm" || robot_type_str == "WHEEL_FOOT_WITH_ARM")
  {
    return RobotType::WHEEL_FOOT_WITH_ARM;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown robot type string: %s.", robot_type_str.c_str());
    throw std::runtime_error("Unknown robot type string.");
  }
}

void Tron1SimNode::build_low_state_msg()
{
  double sim_time = sim_->d_->time;
  low_state_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
  low_state_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg_.header.stamp.sec) * 1e9);
  low_state_msg_.header.frame_id = "sim_time";

  if (robot_type_ == RobotType::POINT_FOOT ||
      robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    for (size_t i = 0; i < 6; i++)
    {
      low_state_msg_.motor_state[i].q = joint_pos_[i];
      low_state_msg_.motor_state[i].dq = joint_vel_[i];
      low_state_msg_.motor_state[i].tau = joint_torque_[i];
    }
  }
  else{
    for (size_t i = 0; i < 8; i++)
    {
      low_state_msg_.motor_state[i].q = joint_pos_[i];
      low_state_msg_.motor_state[i].dq = joint_vel_[i];
      low_state_msg_.motor_state[i].tau = joint_torque_[i];
    }
  }

  // manipulator and gripper
  if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    manipulator_low_state_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
    manipulator_low_state_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg_.header.stamp.sec) * 1e9);
    manipulator_low_state_msg_.header.frame_id = "sim_time";
    gripper_state_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
    gripper_state_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg_.header.stamp.sec) * 1e9);
    gripper_state_msg_.header.frame_id = "sim_time";
    for (size_t i = 0; i < 6; i++)
    {
      manipulator_low_state_msg_.motor_state[i].q = joint_pos_[6 + i];
      manipulator_low_state_msg_.motor_state[i].dq = joint_vel_[6 + i];
      manipulator_low_state_msg_.motor_state[i].tau = joint_torque_[6 + i];
    }
    gripper_state_msg_.gripper_state.q = joint_pos_[12];
    gripper_state_msg_.gripper_state.dq = joint_vel_[12];
    gripper_state_msg_.gripper_state.tau = joint_torque_[12];
  }
  else if (robot_type_ == RobotType::FLAT_FOOT_WITH_ARM ||
           robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
  {
    manipulator_low_state_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
    manipulator_low_state_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg_.header.stamp.sec) * 1e9);
    manipulator_low_state_msg_.header.frame_id = "sim_time";
    gripper_state_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
    gripper_state_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg_.header.stamp.sec) * 1e9);
    gripper_state_msg_.header.frame_id = "sim_time";
    for (size_t i = 0; i < 6; i++)
    {
      manipulator_low_state_msg_.motor_state[i].q = joint_pos_[8 + i];
      manipulator_low_state_msg_.motor_state[i].dq = joint_vel_[8 + i];
      manipulator_low_state_msg_.motor_state[i].tau = joint_torque_[8 + i];
    }
    gripper_state_msg_.gripper_state.q = joint_pos_[14];
    gripper_state_msg_.gripper_state.dq = joint_vel_[14];
    gripper_state_msg_.gripper_state.tau = joint_torque_[14];
  }

  for (int i = 0; i < 2; i++)
  {
    low_state_msg_.contact_state[i].contact = contact_[i];
  }

  low_state_msg_.imu.linear_acceleration.x = accelerom_[0];
  low_state_msg_.imu.linear_acceleration.y = accelerom_[1];
  low_state_msg_.imu.linear_acceleration.z = accelerom_[2];
  low_state_msg_.imu.angular_velocity.x = gyro_[0];
  low_state_msg_.imu.angular_velocity.y = gyro_[1];
  low_state_msg_.imu.angular_velocity.z = gyro_[2];
  low_state_msg_.imu.orientation.w = quat_[0];
  low_state_msg_.imu.orientation.x = quat_[1];
  low_state_msg_.imu.orientation.y = quat_[2];
  low_state_msg_.imu.orientation.z = quat_[3];
}

void Tron1SimNode::callback_low_state()
{
  // get the sensor data
  if (sim_->d_)
  {
    // get the data
    const int idx_imu_quat = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "imu_quat")];
    const int idx_imu_gyro = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "imu_gyro")];
    const int idx_imu_accele = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "imu_acc")];
    const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "abad_L_pos")];
    const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "abad_L_vel")];
    const int idx_joint_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "abad_L_tau")];
    const int idx_contact = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "L_contact_sensor")];

    quat_[0] = sim_->d_->sensordata[idx_imu_quat + 0];  // w
    quat_[1] = sim_->d_->sensordata[idx_imu_quat + 1];  // x
    quat_[2] = sim_->d_->sensordata[idx_imu_quat + 2];  // y
    quat_[3] = sim_->d_->sensordata[idx_imu_quat + 3];  // z
    accelerom_[0] = sim_->d_->sensordata[idx_imu_accele + 0];
    accelerom_[1] = sim_->d_->sensordata[idx_imu_accele + 1];
    accelerom_[2] = sim_->d_->sensordata[idx_imu_accele + 2];
    gyro_[0] = sim_->d_->sensordata[idx_imu_gyro + 0];
    gyro_[1] = sim_->d_->sensordata[idx_imu_gyro + 1];
    gyro_[2] = sim_->d_->sensordata[idx_imu_gyro + 2];

    for (size_t i = 0; i < joint_pos_.size(); i++)
    {
      joint_pos_[i] = sim_->d_->sensordata[idx_joint_pos + i];
      joint_vel_[i] = sim_->d_->sensordata[idx_joint_vel + i];
      joint_torque_[i] = sim_->d_->sensordata[idx_joint_torque + i];
    }

    // touch sensor apply for both point foot, flat foot and wheel foot with different volume of site
    for (int i = 0; i < 2; i++)
    {
      if (sim_->d_->sensordata[idx_contact + i] > 0.0)
      {
        contact_[i] = true;
      }
      else
      {
        contact_[i] = false;
      }
    }

    this->build_low_state_msg();

    low_state_pub_ptr_->publish(low_state_msg_);
    if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM ||
        robot_type_ == RobotType::FLAT_FOOT_WITH_ARM ||
        robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
    {
      manipulator_state_pub_ptr_->publish(manipulator_low_state_msg_);
      gripper_state_pub_ptr_->publish(gripper_state_msg_);
    }
  }
}

void Tron1SimNode::callback_low_cmd(const humanoid_msgs::msg::LowCmd::SharedPtr msg)
{
  if (sim_->d_)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    // store values
    if (robot_type_ == RobotType::POINT_FOOT ||
        robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
    {
      for (size_t i = 0; i < 6; i++)
      {
        mode_[i] = msg->motor_cmd[i].mode;
        cmd_torque_[i] = msg->motor_cmd[i].tau;
        cmd_pos_[i] = msg->motor_cmd[i].q;
        cmd_vel_[i] = msg->motor_cmd[i].dq;
        cmd_kp_[i] = msg->motor_cmd[i].kp;
        cmd_kd_[i] = msg->motor_cmd[i].kd;
      }
    }
    else
    {
      for (size_t i = 0; i < 8; i++)
      {
        mode_[i] = msg->motor_cmd[i].mode;
        cmd_torque_[i] = msg->motor_cmd[i].tau;
        cmd_pos_[i] = msg->motor_cmd[i].q;
        cmd_vel_[i] = msg->motor_cmd[i].dq;
        cmd_kp_[i] = msg->motor_cmd[i].kp;
        cmd_kd_[i] = msg->motor_cmd[i].kd;
      }
    }

    // apply the motor commands
    // assume actuator orders of position -> velocity -> torque
    for (size_t i = 0; i < joint_pos_.size(); i++)
    {
      switch (mode_[i])
      {
        case uint8_t(0):  // no control
          // sim_->d_->ctrl[i] = 0.0;
          break;
        case uint8_t(1):                                             // position control
          sim_->m_->actuator_gainprm[i * mjNGAIN + 0] = cmd_kp_[i];  // set kp
          sim_->m_->actuator_biasprm[i * mjNBIAS + 1] = -cmd_kp_[i];
          sim_->m_->actuator_biasprm[i * mjNBIAS + 2] = -cmd_kd_[i];  // set kd
          sim_->d_->ctrl[i] = cmd_pos_[i];
          break;
        case uint8_t(2):  // velocity control
          sim_->m_->actuator_gainprm[(joint_pos_.size() + i) * mjNGAIN + 0] = cmd_kd_[i];
          sim_->m_->actuator_biasprm[(joint_pos_.size() + i) * mjNBIAS + 2] = -cmd_kd_[i];
          sim_->d_->ctrl[i + joint_pos_.size()] = cmd_vel_[i];
          break;
        case uint8_t(3):  // torque control
          sim_->d_->ctrl[i + 2 * joint_pos_.size()] = cmd_torque_[i];
          break;
        case uint8_t(4):                                             // torque + pd
          sim_->m_->actuator_gainprm[i * mjNGAIN + 0] = cmd_kp_[i];  // set kp
          sim_->m_->actuator_biasprm[i * mjNBIAS + 1] = -cmd_kp_[i];
          sim_->m_->actuator_biasprm[i * mjNBIAS + 2] = -cmd_kd_[i];  // set kd
          sim_->m_->actuator_gainprm[(joint_pos_.size() + i) * mjNGAIN + 0] = cmd_kd_[i];
          sim_->m_->actuator_biasprm[(joint_pos_.size() + i) * mjNBIAS + 2] = 0.0;
          sim_->d_->ctrl[i] = cmd_pos_[i];
          sim_->d_->ctrl[i + joint_pos_.size()] = cmd_vel_[i];
          sim_->d_->ctrl[i + 2 * joint_pos_.size()] = cmd_torque_[i];
          break;
        case uint8_t(5):  // actuator network
          RCLCPP_ERROR(this->get_logger(), "Actuator network mode not implemented yet.");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown control mode: %d for joint %zu", msg->motor_cmd[i].mode, i);
          break;
      }
    }
  }
}

void Tron1SimNode::callback_manipulator_low_cmd(const manipulator_msgs::msg::LowCmd::SharedPtr msg)
{
  if (robot_type_ == RobotType::POINT_FOOT ||
      robot_type_ == RobotType::FLAT_FOOT ||
      robot_type_ == RobotType::WHEEL_FOOT)
  {
    RCLCPP_ERROR(this->get_logger(), "Received manipulator command for robot type without arm.");
    return;
  }

  if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    for (size_t i = 0; i < 6; i++)
    {
      mode_[6 + i] = msg->motor_cmd[i].mode;
      cmd_torque_[6 + i] = msg->motor_cmd[i].tau;
      cmd_pos_[6 + i] = msg->motor_cmd[i].q;
      cmd_vel_[6 + i] = msg->motor_cmd[i].dq;
      cmd_kp_[6 + i] = msg->motor_cmd[i].kp;
      cmd_kd_[6 + i] = msg->motor_cmd[i].kd;
    }
  }
  else if (robot_type_ == RobotType::FLAT_FOOT_WITH_ARM || robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    for (size_t i = 0; i < 6; i++)
    {
      mode_[8 + i] = msg->motor_cmd[i].mode;
      cmd_torque_[8 + i] = msg->motor_cmd[i].tau;
      cmd_pos_[8 + i] = msg->motor_cmd[i].q;
      cmd_vel_[8 + i] = msg->motor_cmd[i].dq;
      cmd_kp_[8 + i] = msg->motor_cmd[i].kp;
      cmd_kd_[8 + i] = msg->motor_cmd[i].kd;
    }
  }
}

void Tron1SimNode::callback_gripper_low_cmd(const end_effector_msgs::msg::GripperCmd::SharedPtr msg)
{
  if (robot_type_ == RobotType::POINT_FOOT ||
      robot_type_ == RobotType::FLAT_FOOT ||
      robot_type_ == RobotType::WHEEL_FOOT)
  {
    RCLCPP_ERROR(this->get_logger(), "Received manipulator command for robot type without arm.");
    return;
  }

  if (!gripper_gains_from_xml_loaded_)
  {
    load_gripper_gains_from_xml();
  }

  const float gripper_kp = gripper_gains_from_xml_loaded_ ? gripper_kp_from_xml_ : msg->gripper_cmd.kp;
  const float gripper_kd = gripper_gains_from_xml_loaded_ ? gripper_kd_from_xml_ : msg->gripper_cmd.kd;

  if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    mode_[12] = msg->gripper_cmd.mode;
    cmd_torque_[12] = msg->gripper_cmd.tau;
    cmd_pos_[12] = msg->gripper_cmd.q;
    cmd_vel_[12] = msg->gripper_cmd.dq;
    cmd_kp_[12] = gripper_kp;
    cmd_kd_[12] = gripper_kd;
  }
  else if (robot_type_ == RobotType::FLAT_FOOT_WITH_ARM || robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    mode_[14] = msg->gripper_cmd.mode;
    cmd_torque_[14] = msg->gripper_cmd.tau;
    cmd_pos_[14] = msg->gripper_cmd.q;
    cmd_vel_[14] = msg->gripper_cmd.dq;
    cmd_kp_[14] = gripper_kp;
    cmd_kd_[14] = gripper_kd;
  }
}

Tron1SimGroundTruth::Tron1SimGroundTruth()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  ground_truth_pub_ptr_ = this->create_publisher<humanoid_msgs::msg::HumanoidEst>("/tron1/humanoid_est", qos);

  timers_.emplace_back(this->create_wall_timer(2ms, std::bind(&Tron1SimGroundTruth::ground_truth_callback, this)));

  ground_truth_msg_.motor_state.resize(8);

  reset_params();
}

void Tron1SimGroundTruth::reset_params()
{
  // reset all the params
  Tron1SimNode::reset_params();
  // ground truth
  pos_truth_ = {0.0, 0.0, 0.0};        // x, y, z
  ori_truth_ = {1.0, 0.0, 0.0, 0.0};   // w, x, y, z
  lin_vel_truth_ = {0.0, 0.0, 0.0};    // vx, vy, vz
  lin_accel_truth_ = {0.0, 0.0, 0.0};  // ax, ay, az
  ang_vel_truth_ = {0.0, 0.0, 0.0};    // wx, wy, wz
  ang_acc_truth_ = {0.0, 0.0, 0.0};    // alphax, alphay, alphaz
}

void Tron1SimGroundTruth::load_ros2_params()
{
  Tron1SimNode::load_ros2_params();

  // if type is point foot, low the q related number to 6
  if (robot_type_ == RobotType::POINT_FOOT)
  {
    ground_truth_msg_.motor_state.resize(6);
  }
  // if type is with arm then add 7 joint for manipulator and gripper
  else if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    ground_truth_msg_.motor_state.resize(13);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
    manipulator_ground_truth_pub_ptr_ = this->create_publisher<manipulator_msgs::msg::ManipulatorEst>("/airbot_play/manipulator_est", qos);
    manipulator_ground_truth_msg_.motor_state.resize(6);
  }
  else if (robot_type_ == RobotType::FLAT_FOOT_WITH_ARM ||
           robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
  {
    ground_truth_msg_.motor_state.resize(15);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
    manipulator_ground_truth_pub_ptr_ = this->create_publisher<manipulator_msgs::msg::ManipulatorEst>("/airbot_play/manipulator_est", qos);
    manipulator_ground_truth_msg_.motor_state.resize(6);
  }
}

void Tron1SimGroundTruth::build_ground_truth_msg()
{
  double sim_time = sim_->d_->time;
  ground_truth_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
  ground_truth_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - ground_truth_msg_.header.stamp.sec) * 1e9);
  ground_truth_msg_.header.frame_id = "sim_time";

  if (robot_type_ == RobotType::POINT_FOOT ||
      robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    for (size_t i = 0; i < 6; i++)
    {
      ground_truth_msg_.motor_state[i].q = joint_pos_[i];
      ground_truth_msg_.motor_state[i].dq = joint_vel_[i];
      ground_truth_msg_.motor_state[i].tau = joint_torque_[i];
    }
  }
  else
  {
    for (size_t i = 0; i < 8; i++)
    {
      ground_truth_msg_.motor_state[i].q = joint_pos_[i];
      ground_truth_msg_.motor_state[i].dq = joint_vel_[i];
      ground_truth_msg_.motor_state[i].tau = joint_torque_[i];
    }
  }

  if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
  {
    manipulator_ground_truth_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
    manipulator_ground_truth_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg_.header.stamp.sec) * 1e9);
    manipulator_ground_truth_msg_.header.frame_id = "sim_time";
    for (size_t i = 0; i < 6; i++)
    {
      manipulator_ground_truth_msg_.motor_state[i].q = joint_pos_[6 + i];
      manipulator_ground_truth_msg_.motor_state[i].dq = joint_vel_[6 + i];
      manipulator_ground_truth_msg_.motor_state[i].tau = joint_torque_[6 + i];
    }
  }
  else if (robot_type_ == RobotType::FLAT_FOOT_WITH_ARM ||
           robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
  {
    manipulator_ground_truth_msg_.header.stamp.sec = static_cast<int32_t>(sim_time);
    manipulator_ground_truth_msg_.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg_.header.stamp.sec) * 1e9);
    manipulator_ground_truth_msg_.header.frame_id = "sim_time";
    for (size_t i = 0; i < 6; i++)
    {
      manipulator_ground_truth_msg_.motor_state[i].q = joint_pos_[8 + i];
      manipulator_ground_truth_msg_.motor_state[i].dq = joint_vel_[8 + i];
      manipulator_ground_truth_msg_.motor_state[i].tau = joint_torque_[8 + i];
    }
  }

  for (int i = 0; i < 2; i++)
  {
    ground_truth_msg_.contact_state[i].contact = contact_[i];
  }

  ground_truth_msg_.pose.position.x = pos_truth_[0];
  ground_truth_msg_.pose.position.y = pos_truth_[1];
  ground_truth_msg_.pose.position.z = pos_truth_[2];
  ground_truth_msg_.pose.orientation.w = ori_truth_[0];
  ground_truth_msg_.pose.orientation.x = ori_truth_[1];
  ground_truth_msg_.pose.orientation.y = ori_truth_[2];
  ground_truth_msg_.pose.orientation.z = ori_truth_[3];

  ground_truth_msg_.twist.linear.x = lin_vel_truth_[0];
  ground_truth_msg_.twist.linear.y = lin_vel_truth_[1];
  ground_truth_msg_.twist.linear.z = lin_vel_truth_[2];
  ground_truth_msg_.twist.angular.x = ang_vel_truth_[0];
  ground_truth_msg_.twist.angular.y = ang_vel_truth_[1];
  ground_truth_msg_.twist.angular.z = ang_vel_truth_[2];

  ground_truth_msg_.accel.linear.x = lin_accel_truth_[0];
  ground_truth_msg_.accel.linear.y = lin_accel_truth_[1];
  ground_truth_msg_.accel.linear.z = lin_accel_truth_[2];
  ground_truth_msg_.accel.angular.x = ang_acc_truth_[0];
  ground_truth_msg_.accel.angular.y = ang_acc_truth_[1];
  ground_truth_msg_.accel.angular.z = ang_acc_truth_[2];

  ground_truth_msg_.contact_wrench[0].wrench.force.x = contact_wrenches_[0][0];  // L
  ground_truth_msg_.contact_wrench[0].wrench.force.y = contact_wrenches_[0][1];
  ground_truth_msg_.contact_wrench[0].wrench.force.z = contact_wrenches_[0][2];
  ground_truth_msg_.contact_wrench[0].wrench.torque.x = contact_wrenches_[0][3];
  ground_truth_msg_.contact_wrench[0].wrench.torque.y = contact_wrenches_[0][4];
  ground_truth_msg_.contact_wrench[0].wrench.torque.z = contact_wrenches_[0][5];
  ground_truth_msg_.contact_wrench[1].wrench.force.x = contact_wrenches_[1][0];  // R
  ground_truth_msg_.contact_wrench[1].wrench.force.y = contact_wrenches_[1][1];
  ground_truth_msg_.contact_wrench[1].wrench.force.z = contact_wrenches_[1][2];
  ground_truth_msg_.contact_wrench[1].wrench.torque.x = contact_wrenches_[1][3];
  ground_truth_msg_.contact_wrench[1].wrench.torque.y = contact_wrenches_[1][4];
  ground_truth_msg_.contact_wrench[1].wrench.torque.z = contact_wrenches_[1][5];
}

void Tron1SimGroundTruth::ground_truth_callback()
{
  if (sim_->d_)
  {
    const int idx_pos_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_pos")];
    const int idx_orien_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_quat")];
    const int idx_linvel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_linvel")];
    const int idx_angvel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_angvel")];
    const int idx_linaccel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_linacc")];
    const int idx_angaccel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_angacc")];
    const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "abad_L_pos")];
    const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "abad_L_vel")];
    const int idx_joint_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "abad_L_tau")];
    const int idx_contact = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "L_contact_sensor")];
    const int idx_contact_force = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "L_grf")];
    const int idx_contact_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "L_grm")];
    const int idx_contact_orient = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "L_foot_quat")];

    // position
    pos_truth_[0] = sim_->d_->sensordata[idx_pos_truth + 0];
    pos_truth_[1] = sim_->d_->sensordata[idx_pos_truth + 1];
    pos_truth_[2] = sim_->d_->sensordata[idx_pos_truth + 2];
    // orientation
    ori_truth_[0] = sim_->d_->sensordata[idx_orien_truth + 0];
    ori_truth_[1] = sim_->d_->sensordata[idx_orien_truth + 1];
    ori_truth_[2] = sim_->d_->sensordata[idx_orien_truth + 2];
    ori_truth_[3] = sim_->d_->sensordata[idx_orien_truth + 3];
    // linear velocity
    lin_vel_truth_[0] = sim_->d_->sensordata[idx_linvel_truth + 0];
    lin_vel_truth_[1] = sim_->d_->sensordata[idx_linvel_truth + 1];
    lin_vel_truth_[2] = sim_->d_->sensordata[idx_linvel_truth + 2];
    // angular velocity
    ang_vel_truth_[0] = sim_->d_->sensordata[idx_angvel_truth + 0];
    ang_vel_truth_[1] = sim_->d_->sensordata[idx_angvel_truth + 1];
    ang_vel_truth_[2] = sim_->d_->sensordata[idx_angvel_truth + 2];
    // linear acceleration
    lin_accel_truth_[0] = sim_->d_->sensordata[idx_linaccel_truth + 0];
    lin_accel_truth_[1] = sim_->d_->sensordata[idx_linaccel_truth + 1];
    lin_accel_truth_[2] = sim_->d_->sensordata[idx_linaccel_truth + 2];
    // angular acceleration
    ang_acc_truth_[0] = sim_->d_->sensordata[idx_angaccel_truth + 0];
    ang_acc_truth_[1] = sim_->d_->sensordata[idx_angaccel_truth + 1];
    ang_acc_truth_[2] = sim_->d_->sensordata[idx_angaccel_truth + 2];

    for (size_t i = 0; i < joint_pos_.size(); i++)
    {
      joint_pos_[i] = sim_->d_->sensordata[idx_joint_pos + i];
      joint_vel_[i] = sim_->d_->sensordata[idx_joint_vel + i];
      joint_torque_[i] = sim_->d_->sensordata[idx_joint_torque + i];
    }

    // touch sensor apply for both point foot, flat foot and wheel foot with different volume of site
    for (int i = 0; i < 2; i++)
    {
      if (sim_->d_->sensordata[idx_contact + i] > 0.0)
      {
        contact_[i] = true;
      }
      else
      {
        contact_[i] = false;
      }
    }

    // contact wrenches
    mjtNum foot_quat[2][4];
    for (size_t foot = 0; foot < contact_.size(); foot++)
    {
      foot_quat[foot][0] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 0];
      foot_quat[foot][1] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 1];
      foot_quat[foot][2] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 2];
      foot_quat[foot][3] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 3];
    }
    mjtNum foot_force_site[2][3];
    mjtNum foot_force_world[2][3];
    mjtNum foot_torque_site[2][3];
    mjtNum foot_torque_world[2][3];

    for (size_t foot = 0; foot < contact_.size(); foot++)
    {
      // NOTE1: grf sensor senses the force exerted on the child body by the parent body
      // when the child body is almost massless, it = -grf
      foot_force_site[foot][0] = -sim_->d_->sensordata[idx_contact_force + foot * 3 + 0];
      foot_force_site[foot][1] = -sim_->d_->sensordata[idx_contact_force + foot * 3 + 1];
      foot_force_site[foot][2] = -sim_->d_->sensordata[idx_contact_force + foot * 3 + 2];

      // Only get torque for flat foot types
      if (robot_type_ == RobotType::FLAT_FOOT || robot_type_ == RobotType::FLAT_FOOT_WITH_ARM)
      {
        foot_torque_site[foot][0] = -sim_->d_->sensordata[idx_contact_torque + foot * 3 + 0];
        foot_torque_site[foot][1] = -sim_->d_->sensordata[idx_contact_torque + foot * 3 + 1];
        foot_torque_site[foot][2] = -sim_->d_->sensordata[idx_contact_torque + foot * 3 + 2];
      }

      mju_rotVecQuat(foot_force_world[foot], foot_force_site[foot], foot_quat[foot]);

      if (robot_type_ == RobotType::FLAT_FOOT || robot_type_ == RobotType::FLAT_FOOT_WITH_ARM)
      {
        mju_rotVecQuat(foot_torque_world[foot], foot_torque_site[foot], foot_quat[foot]);
      }
    }

    // Point foot: only force
    if (robot_type_ == RobotType::POINT_FOOT || robot_type_ == RobotType::POINT_FOOT_WITH_ARM)
    {
      contact_wrenches_[0][0] = foot_force_world[0][0];  // L
      contact_wrenches_[0][1] = foot_force_world[0][1];
      contact_wrenches_[0][2] = foot_force_world[0][2];
      contact_wrenches_[0][3] = 0.0;
      contact_wrenches_[0][4] = 0.0;
      contact_wrenches_[0][5] = 0.0;
      contact_wrenches_[1][0] = foot_force_world[1][0];  // R
      contact_wrenches_[1][1] = foot_force_world[1][1];
      contact_wrenches_[1][2] = foot_force_world[1][2];
      contact_wrenches_[1][3] = 0.0;
      contact_wrenches_[1][4] = 0.0;
      contact_wrenches_[1][5] = 0.0;
    }
    // Flat foot: force + torque (full wrench)
    else if (robot_type_ == RobotType::FLAT_FOOT || robot_type_ == RobotType::FLAT_FOOT_WITH_ARM)
    {
      contact_wrenches_[0][0] = foot_force_world[0][0];  // L
      contact_wrenches_[0][1] = foot_force_world[0][1];
      contact_wrenches_[0][2] = foot_force_world[0][2];
      contact_wrenches_[0][3] = foot_torque_world[0][0];
      contact_wrenches_[0][4] = foot_torque_world[0][1];
      contact_wrenches_[0][5] = foot_torque_world[0][2];
      contact_wrenches_[1][0] = foot_force_world[1][0];  // R
      contact_wrenches_[1][1] = foot_force_world[1][1];
      contact_wrenches_[1][2] = foot_force_world[1][2];
      contact_wrenches_[1][3] = foot_torque_world[1][0];
      contact_wrenches_[1][4] = foot_torque_world[1][1];
      contact_wrenches_[1][5] = foot_torque_world[1][2];
    }
    // Wheel foot: do nothing (set all to zero)
    else if (robot_type_ == RobotType::WHEEL_FOOT || robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
    {
      contact_wrenches_[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      contact_wrenches_[1] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    this->build_ground_truth_msg();

    ground_truth_pub_ptr_->publish(ground_truth_msg_);
    if (robot_type_ == RobotType::POINT_FOOT_WITH_ARM ||
        robot_type_ == RobotType::FLAT_FOOT_WITH_ARM ||
        robot_type_ == RobotType::WHEEL_FOOT_WITH_ARM)
    {
      manipulator_ground_truth_pub_ptr_->publish(manipulator_ground_truth_msg_);
    }
  }
}

}  // namespace mujoco_sim

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_sim::Tron1SimNode, mujoco_sim::MujocoSimNodeBase)
PLUGINLIB_EXPORT_CLASS(mujoco_sim::Tron1SimGroundTruth, mujoco_sim::MujocoSimNodeBase)
