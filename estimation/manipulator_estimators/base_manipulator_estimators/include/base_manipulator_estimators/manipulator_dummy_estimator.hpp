#pragma once
#ifndef MANIPULATOR_DUMMY_ESTIMATOR_HPP__
#define MANIPULATOR_DUMMY_ESTIMATOR_HPP__

#include <vector>

#include "base_manipulator_estimators/base_manipulator_estimator.hpp"
#include "manipulator_msgs/msg/manipulator_est.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulator_estimators  // estimator as a chainable controller
{
class ManipulatorDummyEstimator : public BaseManipulatorEstimator
{
  public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  protected:
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;

  std::string node_name_ = "";
  std::string subscribe_topic_name_ = "";
  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<manipulator_msgs::msg::ManipulatorEst>::SharedPtr ManipulatorEst_subscriber_ = nullptr;
};

};  // namespace manipulator_estimators
#endif
