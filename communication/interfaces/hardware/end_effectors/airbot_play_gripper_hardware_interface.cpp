#include "../include/end_effectors/airbot_play_gripper_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
AirbotPlayGripperHardwareInterface::AirbotPlayGripperHardwareInterface() : GripperHardwareInterface()
{
  node_name_ = "airbot_play_gripper_hardware_interface";
  publish_topic_name_ = "/airbot_play_gripper/gripper_cmd";
  subscribe_topic_name_ = "/airbot_play_gripper/gripper_state";
}
hardware_interface::CallbackReturn AirbotPlayGripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return GripperHardwareInterface::on_configure(previous_state);
}

hardware_interface::CallbackReturn AirbotPlayGripperHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayGripperHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayGripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AirbotPlayGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  auto ret = GripperHardwareInterface::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
  std::fill(mode_.begin(), mode_.end(), 4);  // mit mode even though it's usually position control
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AirbotPlayGripperHardwareInterface::export_state_interfaces()
{
  return GripperHardwareInterface::export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface> AirbotPlayGripperHardwareInterface::export_command_interfaces()
{
  return GripperHardwareInterface::export_command_interfaces();
}

hardware_interface::return_type AirbotPlayGripperHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return GripperHardwareInterface::read(time, period);
}

hardware_interface::return_type AirbotPlayGripperHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return GripperHardwareInterface::write(time, period);
}

}; // namespace hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware_interfaces::AirbotPlayGripperHardwareInterface, hardware_interface::SystemInterface)
