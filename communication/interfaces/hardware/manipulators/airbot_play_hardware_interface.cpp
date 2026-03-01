#include "../include/manipulators/airbot_play_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
AirbotPlayHardwareInterface::AirbotPlayHardwareInterface() : ManipulatorHardwareInterface()
{
  node_name_ = "airbot_play_arm_hardware_interface";
  publish_topic_name_ = "/airbot_play/low_cmd";
  subscribe_topic_name_ = "/airbot_play/low_state";
}
hardware_interface::CallbackReturn AirbotPlayHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return ManipulatorHardwareInterface::on_configure(previous_state);
}

hardware_interface::CallbackReturn AirbotPlayHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  auto ret = ManipulatorHardwareInterface::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
  std::fill(mode_.begin(), mode_.end(), 4);  //  mit mode even though it's usually position control
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AirbotPlayHardwareInterface::export_state_interfaces()
{
  return ManipulatorHardwareInterface::export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface> AirbotPlayHardwareInterface::export_command_interfaces()
{
  return ManipulatorHardwareInterface::export_command_interfaces();
}

hardware_interface::return_type AirbotPlayHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return ManipulatorHardwareInterface::read(time, period);
}

hardware_interface::return_type AirbotPlayHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return ManipulatorHardwareInterface::write(time, period);
}

}; // namespace hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware_interfaces::AirbotPlayHardwareInterface, hardware_interface::SystemInterface)
