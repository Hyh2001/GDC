/*
  Implementation for interfacing with airbot_hardware
  SDK version 0.2.9 for airbot_play arm without gripper.
*/
#pragma once
#ifndef AIRBOT_PLAY_ARM_HARDWARE_INTERFACE_HPP
#define AIRBOT_PLAY_ARM_HARDWARE_INTERFACE_HPP

#include "base_hardware_interfaces/base_manipulator_hardware_interface.hpp"
#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"

namespace airbot_play_hardware_interfaces
{
using namespace airbot::hardware;

class AirbotPlayArmHardwareInterface : public base_hardware_interfaces::BaseManipulatorHardwareInterface
{
  public:
  AirbotPlayArmHardwareInterface();
  ~AirbotPlayArmHardwareInterface();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  bool check_hardware();

  void read() override;
  void write() override;
  void reset() override;

  protected:
  void callback_low_cmd(const manipulator_msgs::msg::LowCmd::SharedPtr msg);
  void publish_low_state();

  manipulator_msgs::msg::LowState low_state_msg_;
  bool start_control_ = false;

  // sdk related
  std::string interface_;
  std::unique_ptr<airbot::hardware::AsioExecutor> arm_exec_;
  std::unique_ptr<airbot::hardware::Arm<6>> arm_;
};

}  // namespace airbot_play_hardware_interfaces

#endif
