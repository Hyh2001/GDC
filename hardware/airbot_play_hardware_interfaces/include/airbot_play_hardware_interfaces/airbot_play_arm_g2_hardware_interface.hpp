/*
  Implementation for interfacing with airbot_hardware
  SDK version 0.2.9 for airbot_play arm with g2 gripper.
*/
#pragma once
#ifndef AIRBOT_PLAY_ARM_G2_HARDWARE_INTERFACE_HPP
#define AIRBOT_PLAY_ARM_G2_HARDWARE_INTERFACE_HPP

#include "base_hardware_interfaces/base_manipulator_hardware_interface.hpp"
#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/eef.hpp"

namespace airbot_play_hardware_interfaces
{
using namespace airbot::hardware;

class AirbotPlayArmG2HardwareInterface : public base_hardware_interfaces::BaseManipulatorHardwareInterface
{
  public:
  AirbotPlayArmG2HardwareInterface();
  ~AirbotPlayArmG2HardwareInterface();

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

  // mode related
  bool use_mit_mode_{false};
  std::array<double, 6> pvt_max_velocity_{airbot::hardware::Arm<6>::DEFAULT_MAX_VEL};
  std::array<double, 6> pvt_max_effort_{airbot::hardware::Arm<6>::DEFAULT_MAX_EFF};

  // sdk related
  std::string interface_;
  std::unique_ptr<airbot::hardware::AsioExecutor> arm_exec_;
  std::unique_ptr<airbot::hardware::AsioExecutor> eef_exec_;
  std::unique_ptr<airbot::hardware::Arm<6>> arm_;
  std::unique_ptr<airbot::hardware::EEF<1>> eef_;
};

} // namespace airbot_play_hardware_interfaces

#endif
