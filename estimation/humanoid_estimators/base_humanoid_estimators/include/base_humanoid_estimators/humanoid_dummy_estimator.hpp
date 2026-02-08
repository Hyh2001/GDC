#pragma once
#include <vector>

#include "base_humanoid_estimators/base_humanoid_estimator.hpp"
#include "humanoid_msgs/msg/humanoid_est.hpp"
#include "rclcpp/rclcpp.hpp"

namespace humanoid_estimators  // estimator as a chainable controller
{
class HumanoidDummyEstimator : public BaseHumanoidEstimator
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
  rclcpp::Subscription<humanoid_msgs::msg::HumanoidEst>::SharedPtr HumanoidEst_subscriber_ = nullptr;
};

};  // namespace humanoid_estimators
