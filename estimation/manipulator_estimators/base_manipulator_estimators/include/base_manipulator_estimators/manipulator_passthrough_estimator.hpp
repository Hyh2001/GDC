#pragma once
#ifndef MANIPULATOR_PASSTHROUGH_ESTIMATOR_
#define MANIPULATOR_PASSTHROUGH_ESTIMATOR_

#include "base_manipulator_estimators/base_manipulator_estimator.hpp"

namespace manipulator_estimators
{
class ManipulatorPassthroughEstimator : public BaseManipulatorEstimator
{
  public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  protected:
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
};

}


#endif
