#pragma once
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "pend_msgs/msg/pendulum_est.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pendulum_estimators  // estimator as a chainable controller
{
class DummyEstimator : public controller_interface::ChainableControllerInterface
{
  public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  protected:
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;

  // sensor readings
  pend_msgs::msg::PendulumEst latest_est_msg_;
  double joint_pos_ = 0.0;
  double joint_vel_ = 0.0;
  double joint_tau_ = 0.0;
  std::array<double, 2> tip_pos_ = {0.0, 0.0};  // y, z
  std::array<double, 2> tip_vel_ = {0.0, 0.0};  // vy, vz

  // ros2 related
  std::string node_name_ = "";
  std::string subscribe_topic_name_ = "";
  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<pend_msgs::msg::PendulumEst>::SharedPtr PendulumEst_subscriber_ = nullptr;
};

};  // namespace pendulum_estimators
