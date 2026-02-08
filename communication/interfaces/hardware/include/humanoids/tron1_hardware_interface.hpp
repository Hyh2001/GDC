#pragma once
#ifndef TRON1_HARDWARE_INTERFACE_HPP
#define TRON1_HARDWARE_INTERFACE_HPP

#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "humanoid_hardware_interface.hpp"
#include "humanoid_msgs/msg/low_cmd.hpp"
#include "humanoid_msgs/msg/low_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace hardware_interfaces
{
class Tron1HardwareInterface : public hardware_interfaces::HumanoidHardwareInterface
{
  public:
  Tron1HardwareInterface();

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

};  // namespace hardware_interfaces

#endif  // TRON1_HARDWARE_INTERFACE_HPP
