#ifndef QUADRUPED_HARDWARE_INTERFACE_HPP
#define QUADRUPED_HARDWARE_INTERFACE_HPP

#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "quadruped_msgs/msg/low_cmd.hpp"
#include "quadruped_msgs/msg/low_state.hpp"
#include "quadruped_msgs/msg/quad_est.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace hardware_interfaces
{
class QuadrupedHardwareInterface : public hardware_interface::SystemInterface
{
  public:
  QuadrupedHardwareInterface();

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
  std::string node_name_ = "quadruped_hardware_interface";
  std::string publish_topic_name_ = "/quadruped/low_cmd";
  std::string subscribe_topic_name_ = "/quadruped/low_state";
  // FR FL RR RL
  hardware_interface::HardwareInfo info_;
  // state interfaces
  std::array<double, 12> joint_pos_{0};
  std::array<double, 12> joint_vel_{0};
  std::array<double, 12> joint_acc_{0};
  std::array<double, 12> joint_tau_{0};
  std::array<double, 4> contact_states_{0};
  std::array<double, 3> gyro_{0};
  std::array<double, 3> accel_{0};
  // command interfaces
  std::array<double, 12> mode_{0};
  std::array<double, 12> joint_pos_command_{0};
  std::array<double, 12> joint_vel_command_{0};
  std::array<double, 12> joint_tau_command_{0};
  std::array<double, 12> joint_kp_{0};
  std::array<double, 12> joint_kd_{0};

  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<quadruped_msgs::msg::LowState>::SharedPtr LowState_subscriber_ = nullptr;
  rclcpp::Publisher<quadruped_msgs::msg::LowCmd>::SharedPtr LowCmd_publisher_ = nullptr;
  realtime_tools::RealtimePublisher<quadruped_msgs::msg::LowCmd>::SharedPtr realtime_LowCmd_publisher_ = nullptr;
};

};  // namespace hardware_interfaces

#endif  // QUADRUPED_HARDWARE_INTERFACE_HPP
