#pragma once
#ifndef BASE_MANIPULATOR_ESTIMATOR_HPP__
#define BASE_MANIPULATOR_ESTIMATOR_HPP__

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/introspection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "base_utils/ros2_control_utils.hpp"

namespace manipulator_estimators  // estimator as a chainable controller
{
class BaseManipulatorEstimator : public controller_interface::ChainableControllerInterface
{
  public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  protected:
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration get_joint_state_interface_configuration() const;

  void read_joint_states_from_state_interfaces(std::vector<double>& pos, std::vector<double>& vel,
                                              std::vector<double>& tau) const;

  // sensor reading fields
  std::vector<std::string> joint_names_ = {};
  int num_joints_ = 0;
  std::vector<std::string> joint_state_interface_types_ = {"position", "velocity", "effort"};
  std::vector<std::string> joint_command_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
  std::unordered_map<std::string, std::vector<std::string>> disabled_cmd_ifaces_;
  std::unordered_map<std::string, std::vector<std::string>> disabled_state_ifaces_;

  // readings
  std::vector<double> joint_pos_ = {};
  std::vector<double> joint_vel_ = {};
  std::vector<double> joint_acc_ = {};
  std::vector<double> joint_tau_ = {};

  bool debug_ = false; // debug flag
};
}
#endif
