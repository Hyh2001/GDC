#pragma once

#include "base_controllers/multi_joint_pid.hpp"
#include "base_humanoid_controllers/base_humanoid_controller.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace humanoid_controllers
{

class HumanoidPDController : public BaseHumanoidController
{
  public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // PID controllers
  base_controllers::MultiJointPID pid_controllers_;
  std::vector<double> joint_pos_ref_ = {};
  std::vector<double> joint_vel_ref_ = {};
  std::vector<double> joint_pos_ = {};
  std::vector<double> joint_vel_ = {};
  std::vector<double> kp_gains_ = {};
  std::vector<double> kd_gains_ = {};
};
};  // namespace humanoid_controllers
