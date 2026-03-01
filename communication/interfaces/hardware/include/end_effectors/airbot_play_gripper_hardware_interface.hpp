#pragma once
#ifndef AIRBOT_PLAY_GRIPPER_HARDWARE_INTERFACE_HPP
#define AIRBOT_PLAY_GRIPPER_HARDWARE_INTERFACE_HPP

#include "gripper_hardware_interface.hpp"

namespace hardware_interfaces
{
class AirbotPlayGripperHardwareInterface : public GripperHardwareInterface
{
public:
  AirbotPlayGripperHardwareInterface();

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:

};

}; // namespace hardware_interfaces


#endif // AIRBOT_PLAY_GRIPPER_HARDWARE_INTERFACE_HPP
