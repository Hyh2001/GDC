#pragma once
#ifndef END_EFFECTOR_HARDWARE_INTERFACE_HPP
#define END_EFFECTOR_HARDWARE_INTERFACE_HPP

#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace hardware_interfaces
{
class EndEffectorHardwareInterface : public hardware_interface::SystemInterface
{
  public:
  EndEffectorHardwareInterface();

  virtual hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  virtual hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  virtual hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  protected:
  std::string node_name_ = "end_effector_hardware_interface";
  std::string publish_topic_name_ = "/end_effector/low_cmd";
  std::string subscribe_topic_name_ = "/end_effector/low_state";
  hardware_interface::HardwareInfo info_;
  // state interfaces
  std::vector<double> joint_pos_{0};
  std::vector<double> joint_vel_{0};
  std::vector<double> joint_acc_{0};
  std::vector<double> joint_tau_{0};
  // command interfaces
  std::vector<uint8_t> mode_{0};
  std::vector<double> joint_pos_command_{0};
  std::vector<double> joint_vel_command_{0};
  std::vector<double> joint_tau_command_{0};
  std::vector<double> joint_kp_{0};
  std::vector<double> joint_kd_{0};

  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

};  // namespace hardware_interfaces

#endif  // END_EFFECTOR_HARDWARE_INTERFACE_HPP
