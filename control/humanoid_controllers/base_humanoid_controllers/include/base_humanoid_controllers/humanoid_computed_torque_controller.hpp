#pragma once

#include "base_controllers/multi_joint_pid.hpp"
#include "base_humanoid_controllers/base_humanoid_controller.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace humanoid_controllers
{
/*
    This controller implements a computed torque controller by cancelling away the
    coupled dynamics and using PID control on the joint space.
    The controller is designed to be used as a reference controller in the humanoid control stack,
    where it receives reference trajectories from the planner and outputs joint commands to the low-level controllers.
    The controller can also be used as a standalone controller for testing and debugging purposes.

    since the robot is underactuated, here the computed torque controller is formulated as a QP problem solved with tsid
*/

class HumanoidComputedTorqueController : public BaseHumanoidController
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

  void formulate();

  std::vector<double> joint_pos_ref_ = {};
  std::vector<double> joint_vel_ref_ = {};
  std::vector<double> joint_pos_ = {};
  std::vector<double> joint_vel_ = {};
  std::vector<double> kp_gains_ = {};
  std::vector<double> kd_gains_ = {};
};
};  // namespace humanoid_controllers
