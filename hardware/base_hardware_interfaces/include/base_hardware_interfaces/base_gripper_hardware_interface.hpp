#pragma once
#ifndef BASE_GRIPPER_HARDWARE_INTERFACE_HPP
#define BASE_GRIPPER_HARDWARE_INTERFACE_HPP

#include <vector>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "end_effector_msgs/msg/gripper_cmd.hpp"
#include "end_effector_msgs/msg/gripper_state.hpp"
#include "base_hardware_interfaces/base_end_efffector_hardware_interface.hpp"

namespace base_hardware_interfaces
{

class BaseGripperHardwareInterfaces : public BaseEndEffectorHardwareInterface
{
  public:
  BaseGripperHardwareInterfaces(const std::string& node_name) : BaseEndEffectorHardwareInterface(node_name)
  {
  }

  virtual ~BaseGripperHardwareInterfaces() = default;

  virtual void read() = 0;
  virtual void write() = 0;
  virtual void reset() = 0;

  protected:
  // low state
  std::vector<double> joint_positions_{0.0};
  std::vector<double> joint_velocities_{0.0};
  std::vector<double> joint_efforts_{0.0};

  // low cmd
  std::vector<uint8_t> mode_{0};
  std::vector<double> joint_position_commands_{0.0};
  std::vector<double> joint_velocity_commands_{0.0};
  std::vector<double> joint_effort_commands_{0.0};
  std::vector<double> joint_kp_gains_{0.0};
  std::vector<double> joint_kd_gains_{0.0};

  // subscriber and publisher (lifecycle versions)
  std::vector<rclcpp::TimerBase::SharedPtr> timers_{};
  rclcpp::Subscription<end_effector_msgs::msg::GripperCmd>::SharedPtr gripper_cmd_sub_ptr_ = nullptr;
  rclcpp::Publisher<end_effector_msgs::msg::GripperState>::SharedPtr gripper_state_pub_ptr_ = nullptr;
  realtime_tools::RealtimePublisher<end_effector_msgs::msg::GripperCmd>::SharedPtr realtime_gripper_cmd_publisher_ = nullptr;
};

} // namespace base_hardware_interfaces

#endif
