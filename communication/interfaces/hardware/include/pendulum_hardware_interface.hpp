#ifndef PENDULUM_HARDWARE_INTERFACE_HPP
#define PENDULUM_HARDWARE_INTERFACE_HPP

#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "pend_msgs/msg/low_cmd.hpp"
#include "pend_msgs/msg/low_state.hpp"
#include "pend_msgs/msg/pendulum_est.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace hardware_interfaces
{
class PendulumHardwareInterface : public hardware_interface::SystemInterface
{
  public:
  PendulumHardwareInterface();

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  protected:
  double pivot_position_ = 0.0;
  double pivot_velocity_ = 0.0;
  double pivot_effort_ = 0.0;
  double tip_sensor_y_ = 0.0;
  double tip_sensor_z_ = 0.0;
  double tip_sensor_vy_ = 0.0;
  double tip_sensor_vz_ = 0.0;
  double pivot_pos_command_ = 0.0;
  double pivot_vel_command_ = 0.0;
  double pivot_effort_command_ = 0.0;
  double kp_command_ = 0.0;
  double kd_command_ = 0.0;

  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<pend_msgs::msg::LowState>::SharedPtr LowState_subscriber_ = nullptr;
  rclcpp::Publisher<pend_msgs::msg::LowCmd>::SharedPtr LowCmd_publisher_ = nullptr;
  realtime_tools::RealtimePublisher<pend_msgs::msg::LowCmd>::SharedPtr realtime_LowCmd_publisher_ = nullptr;
  // rclcpp::Subscription<pendulum_msgs::msg::PendulumEst>::SharedPtr PendulumEst_subscriber_ = nullptr;
};

};  // namespace hardware_interfaces

#endif  // PENDULUM_HARDWARE_INTERFACE_HPP
