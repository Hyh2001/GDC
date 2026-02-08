#pragma once

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace quadruped_controllers
{

class Edamp : public controller_interface::ChainableControllerInterface
{
  public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;

  // FR FL RR RL
  std::array<std::string, 12> joint_names_ = {"FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
                                              "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
                                              "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};
  // state and command interfaces
  std::array<std::string, 3> joint_state_interface_types_ = {"position", "velocity", "effort"};
  std::array<std::string, 5> joint_command_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
  std::vector<std::string> foot_names = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};
  std::vector<std::string> foot_sensor_names = {"state", "force_x", "force_y", "force_z"};
  std::string pos_name = "global_pos";
  std::string ori_name = "orientation";  // wxyz
  std::string lin_vel_name = "global_lin_vel";
  std::string ang_vel_name = "global_ang_vel";
  std::string lin_acc_name = "global_lin_acc";
  std::string ang_acc_name = "global_ang_acc";
};
};  // namespace quadruped_controllers
