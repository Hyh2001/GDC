#pragma once
#ifndef BASE_MANIPULATOR_HARDWARE_INTERFACE_HPP
#define BASE_MANIPULATOR_HARDWARE_INTERFACE_HPP

#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "manipulator_msgs/msg/low_cmd.hpp"
#include "manipulator_msgs/msg/low_state.hpp"

namespace base_hardware_interfaces
{

class BaseManipulatorHardwareInterface : public rclcpp_lifecycle::LifecycleNode
{
  public:
  BaseManipulatorHardwareInterface(const std::string& node_name) : rclcpp_lifecycle::LifecycleNode(node_name)
  {
  }

  virtual ~BaseManipulatorHardwareInterface() = default;

  virtual void read() = 0;
  virtual void write() = 0;
  virtual void reset() = 0;

  protected:
  // low state
  std::vector<double> joint_positions_{};
  std::vector<double> joint_velocities_{};
  std::vector<double> joint_efforts_{};

  // low cmd
  std::vector<uint8_t> mode_{};
  std::vector<double> joint_position_commands_{};
  std::vector<double> joint_velocity_commands_{};
  std::vector<double> joint_effort_commands_{};
  std::vector<double> joint_kp_gains_{};
  std::vector<double> joint_kd_gains_{};

  // subscriber and publisher (lifecycle versions)
  std::vector<rclcpp::TimerBase::SharedPtr> timers_{};
  rclcpp::Subscription<manipulator_msgs::msg::LowCmd>::SharedPtr low_cmd_sub_ptr_ = nullptr;
  rclcpp::Publisher<manipulator_msgs::msg::LowState>::SharedPtr low_state_pub_ptr_ = nullptr;
  realtime_tools::RealtimePublisher<manipulator_msgs::msg::LowState>::SharedPtr realtime_low_state_publisher_ = nullptr;
};




} // namespace base_hardware_interfaces

#endif
