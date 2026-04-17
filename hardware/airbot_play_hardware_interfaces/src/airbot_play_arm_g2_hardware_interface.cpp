#include "airbot_play_hardware_interfaces/airbot_play_arm_g2_hardware_interface.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace airbot_play_hardware_interfaces
{

AirbotPlayArmG2HardwareInterface::AirbotPlayArmG2HardwareInterface()
  : BaseManipulatorHardwareInterface("airbot_play_arm_g2_hardware_interface")
{
}

AirbotPlayArmG2HardwareInterface::~AirbotPlayArmG2HardwareInterface()
{
  arm_->disable();
  arm_->uninit();
  eef_->disable();
  eef_->uninit();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AirbotPlayArmG2HardwareInterface::on_configure(
    const rclcpp_lifecycle::State&)
{
  // subscriber
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
  low_cmd_sub_ptr_ = this->create_subscription<manipulator_msgs::msg::LowCmd>(
      "/airbot_play/low_cmd", qos, std::bind(&AirbotPlayArmG2HardwareInterface::callback_low_cmd, this, std::placeholders::_1));
  // publisher
  low_state_pub_ptr_ = this->create_publisher<manipulator_msgs::msg::LowState>("/airbot_play/low_state", qos);
  realtime_low_state_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<manipulator_msgs::msg::LowState>>(low_state_pub_ptr_);
  // timer
  timers_.emplace_back(this->create_wall_timer(std::chrono::milliseconds(2),
                                               std::bind(&AirbotPlayArmG2HardwareInterface::publish_low_state, this)));

  // init buffers
  joint_positions_.resize(7, 0.0);
  joint_velocities_.resize(7, 0.0);
  joint_efforts_.resize(7, 0.0);
  joint_position_commands_.resize(7, 0.0);
  joint_velocity_commands_.resize(7, 0.0);
  joint_effort_commands_.resize(7, 0.0);
  joint_kp_gains_.resize(7, 0.0);
  joint_kd_gains_.resize(7, 0.0);
  low_state_msg_.motor_state.resize(7); // include gripper

  // sdk
  this->declare_parameter<std::string>("interface", "can0");
  this->get_parameter("interface", interface_);
  arm_exec_ = airbot::hardware::AsioExecutor::create(8);
  eef_exec_ = airbot::hardware::AsioExecutor::create(1);
  arm_ = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                           MotorType::DM, EEFType::NA, MotorType::NA>();
  eef_ = EEF<1>::create<EEFType::G2, MotorType::DM>();

  if (!arm_->init(arm_exec_->get_io_context(), interface_, 500_hz)) {
    RCLCPP_ERROR(this->get_logger(), "Arm Executor initialization failed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  if (!eef_->init(eef_exec_->get_io_context(), interface_, 500_hz)) {
    RCLCPP_ERROR(this->get_logger(), "Gripper Executor initialization failed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  arm_->enable();
  arm_->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  eef_->enable();
  eef_->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  if (arm_->state().is_valid) {
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start Airbot SDK");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (!check_hardware())
  {
    RCLCPP_ERROR(this->get_logger(), "Hardware check failed during configuration.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool AirbotPlayArmG2HardwareInterface::check_hardware()
{
  auto state = arm_->state();
  std::vector<double> pos(state.pos.begin(), state.pos.end());
  std::vector<double> vel(state.vel.begin(), state.vel.end());
  std::vector<double> eff(state.eff.begin(), state.eff.end());
  if (pos.size() != 6 || vel.size() != 6 || eff.size() != 6) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get joint positions from Airbot SDK. Please check the connection and SDK status.");
    return false;
  }
  arm_->pvt({pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  arm_->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  RCLCPP_INFO(this->get_logger(), "Arm control mode set to MIT.");
  return true;
}

void AirbotPlayArmG2HardwareInterface::read()
{
  auto state = arm_->state();
  // RCLCPP_INFO_THROTTLE(
  //   this->get_logger(), *this->get_clock(), 1000,
  //   "valid=%d pos5=%.6f vel5=%.6f",
  //   state.is_valid, state.pos[5], state.vel[5]);
  std::copy(state.pos.begin(), state.pos.end(), joint_positions_.begin());
  std::copy(state.vel.begin(), state.vel.end(), joint_velocities_.begin());
  std::copy(state.eff.begin(), state.eff.end(), joint_efforts_.begin());
  joint_positions_[6] = eef_->state().pos[0];
  joint_velocities_[6] = eef_->state().vel[0];
  joint_efforts_[6] = eef_->state().eff[0];

  low_state_msg_.header.stamp = this->now();
  for (size_t i = 0; i < joint_positions_.size(); ++i)
  {
    low_state_msg_.motor_state[i].q = joint_positions_[i];
    low_state_msg_.motor_state[i].dq = joint_velocities_[i];
    low_state_msg_.motor_state[i].tau = joint_efforts_[i];
  }
  realtime_low_state_publisher_->lock();
  realtime_low_state_publisher_->msg_ = low_state_msg_;
  realtime_low_state_publisher_->unlockAndPublish();

  write(); // put write here to make sure command sending and reading are synchronized
}

void AirbotPlayArmG2HardwareInterface::write()
{
  // send arm commands
  std::array<double, 6> arm_joint_position_commands;
  std::array<double, 6> arm_joint_velocity_commands;
  std::array<double, 6> arm_joint_effort_commands;
  std::array<double, 6> arm_joint_kp_gains;
  std::array<double, 6> arm_joint_kd_gains;
  for (size_t i = 0; i < 6; ++i) {
    arm_joint_position_commands[i] = joint_position_commands_[i];
    arm_joint_velocity_commands[i] = joint_velocity_commands_[i];
    arm_joint_effort_commands[i] = joint_effort_commands_[i];
    arm_joint_kp_gains[i] = joint_kp_gains_[i];
    arm_joint_kd_gains[i] = joint_kd_gains_[i];
  }
  arm_->mit(arm_joint_position_commands,
            arm_joint_velocity_commands,
            arm_joint_effort_commands,
            arm_joint_kp_gains,
            arm_joint_kd_gains);

  // send gripper commands
  double gripper_position_command;
  double gripper_velocity_command;
  double gripper_effort_command;
  double gripper_kp_gain;
  double gripper_kd_gain;

  gripper_position_command = joint_position_commands_[6];
  gripper_velocity_command = joint_velocity_commands_[6];
  gripper_effort_command = joint_effort_commands_[6];
  gripper_kp_gain = joint_kp_gains_[6];
  gripper_kd_gain = joint_kd_gains_[6];
  if(gripper_position_command < 0.0 || gripper_position_command > 0.07) {
    RCLCPP_WARN(this->get_logger(), "Gripper position command out of range: %.3f. Clamping to [0.0, 0.085]", gripper_position_command);
    gripper_position_command = std::clamp(gripper_position_command, 0.0, 0.07);
  }

  EEFCommand<1> eef_cmd;
  eef_cmd.pos[0] = gripper_position_command;
  eef_cmd.vel[0] = gripper_velocity_command;
  eef_cmd.eff[0] = gripper_effort_command;
  eef_ ->pvt(eef_cmd);
  // eef_cmd.mit_kp[0] = gripper_kp_gain;
  // eef_cmd.mit_kd[0] = gripper_kd_gain;
  // eef_->mit(eef_cmd);
}

void AirbotPlayArmG2HardwareInterface::reset()
{
  // sensor readings
  joint_positions_.assign(joint_positions_.size(), 0.0);
  joint_velocities_.assign(joint_velocities_.size(), 0.0);
  joint_efforts_.assign(joint_efforts_.size(), 0.0);

  // commands
  joint_position_commands_.assign(joint_position_commands_.size(), 0.0);
  joint_velocity_commands_.assign(joint_velocity_commands_.size(), 0.0);
  joint_effort_commands_.assign(joint_effort_commands_.size(), 0.0);
  joint_kp_gains_.assign(joint_kp_gains_.size(), 0.0);
  joint_kd_gains_.assign(joint_kd_gains_.size(), 0.0);
}

void AirbotPlayArmG2HardwareInterface::callback_low_cmd(const manipulator_msgs::msg::LowCmd::SharedPtr msg)
{
  // receive motor commands
  for (size_t i = 0; i < joint_position_commands_.size(); ++i)
  {
    joint_position_commands_[i] = msg->motor_cmd[i].q;
    joint_velocity_commands_[i] = msg->motor_cmd[i].dq;
    joint_effort_commands_[i] = msg->motor_cmd[i].tau;
    joint_kp_gains_[i] = msg->motor_cmd[i].kp;
    joint_kd_gains_[i] = msg->motor_cmd[i].kd;
  }
}

void AirbotPlayArmG2HardwareInterface::publish_low_state()
{
  read();
}

} // namespace airbot_play_hardware_interfaces

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<airbot_play_hardware_interfaces::AirbotPlayArmG2HardwareInterface>();
  const auto configured_state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configured_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to transition airbot play arm g2 hardware interface node to 'configured'.");
    rclcpp::shutdown();
    return 1;
  }

  const auto activated_state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activated_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to transition airbot play arm g2 hardware interface node to 'active'.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
