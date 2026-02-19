#pragma once
#ifndef GRIPPER_HARDWARE_INTERFACE_HPP
#define GRIPPER_HARDWARE_INTERFACE_HPP

#include "end_effector_msgs/msg/gripper_cmd.hpp"
#include "end_effector_msgs/msg/gripper_state.hpp"
#include "interfaces/hardware/include/end_effectors/end_effector_hardware_interface.hpp"

namespace hardware_interfaces
{
class GripperHardwareInterface : public EndEffectorHardwareInterface
{
  public:
  GripperHardwareInterface();

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
  std::string node_name_ = "gripper_hardware_interface";
  std::string publish_topic_name_ = "/gripper/gripper_cmd";
  std::string subscribe_topic_name_ = "/gripper/gripper_state";

  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<end_effector_msgs::msg::GripperState>::SharedPtr GripperState_subscriber_ = nullptr;
  rclcpp::Publisher<end_effector_msgs::msg::GripperCmd>::SharedPtr GripperCmd_publisher_ = nullptr;
  realtime_tools::RealtimePublisher<end_effector_msgs::msg::GripperCmd>::SharedPtr realtime_GripperCmd_publisher_ = nullptr;
};

};  // namespace hardware_interfaces


#endif  // GRIPPER_HARDWARE_INTERFACE_HPP
