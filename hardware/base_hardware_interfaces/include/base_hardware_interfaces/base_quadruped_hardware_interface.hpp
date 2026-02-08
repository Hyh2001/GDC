#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

#include "quadruped_msgs/msg/low_cmd.hpp"
#include "quadruped_msgs/msg/low_state.hpp"

namespace base_hardware_interfaces
{

class BaseQuadrupedHardwareInterfaces : public rclcpp_lifecycle::LifecycleNode
{
  public:
  BaseQuadrupedHardwareInterfaces(const std::string& node_name) : rclcpp_lifecycle::LifecycleNode(node_name)
  {
  }

  virtual ~BaseQuadrupedHardwareInterfaces() = default;

  virtual void read() = 0;
  virtual void write() = 0;
  virtual void reset() = 0;

  protected:
  // low state
  std::array<double, 12> joint_positions_;
  std::array<double, 12> joint_velocities_;
  std::array<double, 12> joint_efforts_;
  std::array<double, 3> gyro_;
  std::array<double, 3> accelerometer_;
  std::array<bool, 2> contacts_;

  // low cmd
  std::array<double, 12> joint_position_commands_;
  std::array<double, 12> joint_velocity_commands_;
  std::array<double, 12> joint_effort_commands_;
  std::array<double, 12> joint_kp_gains_;
  std::array<double, 12> joint_kd_gains_;

  // subscriber and publisher (lifecycle versions)
  std::vector<rclcpp::TimerBase::SharedPtr> timers_{};
  rclcpp::Subscription<quadruped_msgs::msg::LowCmd>::SharedPtr cmd_sub_ptr_ = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<quadruped_msgs::msg::LowState>::SharedPtr low_state_pub_ptr_ = nullptr;
};

}  // namespace base_hardware_interfaces
