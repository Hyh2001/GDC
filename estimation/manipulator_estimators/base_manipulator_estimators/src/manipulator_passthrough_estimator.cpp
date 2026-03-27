#include "base_manipulator_estimators/manipulator_passthrough_estimator.hpp"

namespace manipulator_estimators
{
controller_interface::CallbackReturn ManipulatorPassthroughEstimator::on_init()
{
  auto ret = BaseManipulatorEstimator::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ManipulatorPassthroughEstimator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn ManipulatorPassthroughEstimator::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ManipulatorPassthroughEstimator::update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period)
{
  BaseManipulatorEstimator::read_joint_states_from_state_interfaces(joint_pos_, joint_vel_, joint_tau_);
  return controller_interface::return_type::OK;
}

controller_interface::return_type ManipulatorPassthroughEstimator::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // Passthrough estimator has no subscriber-fed reference update.
  return controller_interface::return_type::OK;
}

} // namespace manipulator_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(manipulator_estimators::ManipulatorPassthroughEstimator, controller_interface::ChainableControllerInterface);
