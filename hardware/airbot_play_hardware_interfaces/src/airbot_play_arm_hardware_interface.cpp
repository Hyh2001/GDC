#include "airbot_play_hardware_interfaces/airbot_play_arm_hardware_interface.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace airbot_play_hardware_interfaces
{

AirbotPlayArmHardwareInterface::AirbotPlayArmHardwareInterface()
  : BaseManipulatorHardwareInterface("airbot_play_arm_hardware_interface")
{
}

AirbotPlayArmHardwareInterface::~AirbotPlayArmHardwareInterface()
{
  if (arm_)
  {
    arm_->uninit();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AirbotPlayArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State&)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  low_cmd_sub_ptr_ = this->create_subscription<manipulator_msgs::msg::LowCmd>(
      "/airbot_play/low_cmd", qos, std::bind(&AirbotPlayArmHardwareInterface::callback_low_cmd, this, std::placeholders::_1));

  low_state_pub_ptr_ = this->create_publisher<manipulator_msgs::msg::LowState>("/airbot_play/low_state", qos);
  realtime_low_state_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<manipulator_msgs::msg::LowState>>(low_state_pub_ptr_);

  timers_.emplace_back(this->create_wall_timer(std::chrono::milliseconds(2),
                                               std::bind(&AirbotPlayArmHardwareInterface::publish_low_state, this)));

  joint_positions_.resize(6, 0.0);
  joint_velocities_.resize(6, 0.0);
  joint_efforts_.resize(6, 0.0);
  joint_position_commands_.resize(6, 0.0);
  joint_velocity_commands_.resize(6, 0.0);
  joint_effort_commands_.resize(6, 0.0);
  joint_kp_gains_.resize(6, 0.0);
  joint_kd_gains_.resize(6, 0.0);
  low_state_msg_.motor_state.resize(6);

  this->declare_parameter<std::string>("interface", "can0");
  this->get_parameter("interface", interface_);

  arm_exec_ = airbot::hardware::AsioExecutor::create(8);
  arm_ = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                           MotorType::DM, EEFType::NA, MotorType::NA>();

  if (!arm_->init(arm_exec_->get_io_context(), interface_, 250_hz))
  {
    RCLCPP_ERROR(this->get_logger(), "Arm Executor initialization failed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  arm_->enable();
  arm_->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  if (!check_hardware())
  {
    RCLCPP_ERROR(this->get_logger(), "Hardware check failed during configuration.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool AirbotPlayArmHardwareInterface::check_hardware()
{
  auto state = arm_->state();
  if (!state.is_valid)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to start Airbot SDK");
    return false;
  }

  std::vector<double> pos(state.pos.begin(), state.pos.end());
  std::vector<double> vel(state.vel.begin(), state.vel.end());
  std::vector<double> eff(state.eff.begin(), state.eff.end());
  if (pos.size() != 6 || vel.size() != 6 || eff.size() != 6)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to get joint states from Airbot SDK. Please check the connection and SDK status.");
    return false;
  }

  arm_->pvt({pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  return true;
}

void AirbotPlayArmHardwareInterface::read()
{
  auto state = arm_->state();
  std::copy(state.pos.begin(), state.pos.end(), joint_positions_.begin());
  std::copy(state.vel.begin(), state.vel.end(), joint_velocities_.begin());
  std::copy(state.eff.begin(), state.eff.end(), joint_efforts_.begin());

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

  if (start_control_)
  {
    write();
  }
}

void AirbotPlayArmHardwareInterface::write()
{
  std::array<double, 6> arm_joint_position_commands;
  std::array<double, 6> arm_joint_velocity_commands;
  std::array<double, 6> arm_joint_effort_commands;
  std::array<double, 6> arm_joint_kp_gains;
  std::array<double, 6> arm_joint_kd_gains;

  for (size_t i = 0; i < 6; ++i)
  {
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
}

void AirbotPlayArmHardwareInterface::reset()
{
  joint_positions_.assign(joint_positions_.size(), 0.0);
  joint_velocities_.assign(joint_velocities_.size(), 0.0);
  joint_efforts_.assign(joint_efforts_.size(), 0.0);

  joint_position_commands_.assign(joint_position_commands_.size(), 0.0);
  joint_velocity_commands_.assign(joint_velocity_commands_.size(), 0.0);
  joint_effort_commands_.assign(joint_effort_commands_.size(), 0.0);
  joint_kp_gains_.assign(joint_kp_gains_.size(), 0.0);
  joint_kd_gains_.assign(joint_kd_gains_.size(), 0.0);
}

void AirbotPlayArmHardwareInterface::callback_low_cmd(const manipulator_msgs::msg::LowCmd::SharedPtr msg)
{
  for (size_t i = 0; i < joint_position_commands_.size(); ++i)
  {
    joint_position_commands_[i] = msg->motor_cmd[i].q;
    joint_velocity_commands_[i] = msg->motor_cmd[i].dq;
    joint_effort_commands_[i] = msg->motor_cmd[i].tau;
    joint_kp_gains_[i] = msg->motor_cmd[i].kp;
    joint_kd_gains_[i] = msg->motor_cmd[i].kd;
  }

  if (!start_control_)
  {
    arm_->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
    RCLCPP_INFO(this->get_logger(), "Arm control mode set to MIT.");
  }
  start_control_ = true;
}

void AirbotPlayArmHardwareInterface::publish_low_state()
{
  read();
}

}  // namespace airbot_play_hardware_interfaces

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<airbot_play_hardware_interfaces::AirbotPlayArmHardwareInterface>();
  const auto configured_state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configured_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to transition airbot play arm hardware interface node to 'configured'.");
    rclcpp::shutdown();
    return 1;
  }

  const auto activated_state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activated_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to transition airbot play arm hardware interface node to 'active'.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
