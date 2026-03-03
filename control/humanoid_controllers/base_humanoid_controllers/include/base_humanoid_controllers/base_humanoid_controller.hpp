#pragma once

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/introspection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "base_utils/ros2_control_utils.hpp"

namespace humanoid_controllers
{

class BaseHumanoidController : public controller_interface::ChainableControllerInterface
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

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;

  /*
    Interface helpers
  */
  controller_interface::InterfaceConfiguration get_joint_state_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_global_pos_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_ori_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_global_lin_vel_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_global_ang_vel_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_global_lin_acc_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_global_ang_acc_interface_configuration() const;

  controller_interface::InterfaceConfiguration get_contact_state_interface_configuration() const; // contact state

  controller_interface::InterfaceConfiguration get_contact_force_torque_interface_configuration() const; // contact force and torque

  controller_interface::InterfaceConfiguration get_joint_command_interface_configuration() const;

  /*
    Data helpers
  */
  void read_global_pos_from_state_interfaces(std::array<double, 3>& pos);

  void read_ori_from_state_interfaces(std::array<double, 4>& ori);

  void read_global_lin_vel_from_state_interfaces(std::array<double, 3>& lin_vel);

  void read_global_ang_vel_from_state_interfaces(std::array<double, 3>& ang_vel);

  void read_global_lin_acc_from_state_interfaces(std::array<double, 3>& lin_acc);

  void read_global_ang_acc_from_state_interfaces(std::array<double, 3>& ang_acc);

  void read_contact_state_from_state_interfaces(std::array<bool, 2>& contact_state);

  void read_contact_force_torque_from_state_interfaces(std::array<std::array<double, 6>, 2>& contact_force_torque);

  void read_joint_states_from_state_interfaces(std::vector<double>& pos, std::vector<double>& vel,
                                              std::vector<double>& tau) const;

  void write_joint_commands_to_command_interfaces(const std::vector<double>& pos, const std::vector<double>& vel,
                                                  const std::vector<double>& tau, const std::vector<double>& kp,
                                                  const std::vector<double>& kd);
  // predecessors
  std::string estimator_name_ = "";
  std::string planner_name_ = "";
  std::string ref_controller_name_ = "";

  // L -> R
  std::vector<std::string> joint_names_ = {};
  // state and command interfaces
  std::vector<std::string> joint_state_interface_types_ = {"position", "velocity", "effort"};
  std::vector<std::string> joint_command_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
  std::vector<std::string> foot_names_ = {"L_foot", "R_foot"};
  std::vector<std::string> foot_sensor_names_ = {"state",    "force_x",  "force_y", "force_z",
                                                 "torque_x", "torque_y", "torque_z"};
  std::string pos_name_ = "global_pos";
  std::string ori_name_ = "orientation";  // wxyz
  // velocity and acceleration interfaces can be either global or linear
  std::string lin_vel_name_ = "global_lin_vel";
  std::string ang_vel_name_ = "global_ang_vel";
  std::string lin_acc_name_ = "global_lin_acc";
  std::string ang_acc_name_ = "global_ang_acc";

  // debug
  bool debug_ = false;
};
};  // namespace humanoid_controllers
