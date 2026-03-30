#pragma once
#ifndef BASE_MANIPULATOR_CONTROLLERS_HPP__
#define BASE_MANIPULATOR_CONTROLLERS_HPP__

#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <limits>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/introspection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "base_utils/ros2_control_utils.hpp"

namespace manipulator_controllers
{
class BaseManipulatorController : public controller_interface::ChainableControllerInterface
{
  public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration get_joint_state_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_joint_command_interface_configuration() const;

  void read_joint_states_from_state_interfaces(std::vector<double>& pos, std::vector<double>& vel,
                                              std::vector<double>& tau) const;

  void write_joint_commands_to_command_interfaces(const std::vector<double>& pos, const std::vector<double>& vel,
                                                  const std::vector<double>& tau, const std::vector<double>& kp,
                                                  const std::vector<double>& kd);

                                                  // predecessors
  std::string estimator_name_ = "";
  std::string planner_name_ = "";
  std::string ref_controller_name_ = "";

  std::vector<std::string> joint_names_ = {};
  // state and command interfaces
  std::vector<std::string> joint_state_interface_types_ = {"position", "velocity", "effort"};
  std::vector<std::string> joint_command_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
  std::unordered_map<std::string, std::vector<std::string>> disabled_cmd_ifaces_;
  std::unordered_map<std::string, std::vector<std::string>> disabled_state_ifaces_;

  bool debug_ = false;
};
} // namespace manipulator_controllers

#endif // BASE_MANIPULATOR_CONTROLLERS_HPP__
