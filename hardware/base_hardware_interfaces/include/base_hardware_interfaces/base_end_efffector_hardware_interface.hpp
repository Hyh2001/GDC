#pragma once
#ifndef BASE_END_EFFECTOR_HARDWARE_INTERFACE_HPP
#define BASE_END_EFFECTOR_HARDWARE_INTERFACE_HPP

#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace base_hardware_interfaces
{

class BaseEndEffectorHardwareInterface : public rclcpp_lifecycle::LifecycleNode
{
  public:
  BaseEndEffectorHardwareInterface(const std::string& node_name) : rclcpp_lifecycle::LifecycleNode(node_name)
  {
  }

  virtual ~BaseEndEffectorHardwareInterface() = default;

  virtual void read() = 0;
  virtual void write() = 0;
  virtual void reset() = 0;

  protected:
  // low state
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;

  // low cmd
  std::vector<double> mode_{};
  std::vector<double> joint_position_commands_{};
  std::vector<double> joint_velocity_commands_{};
  std::vector<double> joint_effort_commands_{};
  std::vector<double> joint_kp_gains_{};
  std::vector<double> joint_kd_gains_{};
};


} // namespace base_hardware_interfaces


#endif
