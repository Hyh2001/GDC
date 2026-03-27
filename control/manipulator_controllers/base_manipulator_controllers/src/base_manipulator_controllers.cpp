#include "base_manipulator_controllers/base_manipulator_controllers.hpp"

namespace manipulator_controllers
{

controller_interface::CallbackReturn BaseManipulatorController::on_init()
{
  // predecessors
  estimator_name_ = auto_declare<std::string>("estimator_name", estimator_name_);
  planner_name_ = auto_declare<std::string>("planner_name", planner_name_);
  ref_controller_name_ = auto_declare<std::string>("ref_controller_name", ref_controller_name_);

  // interfaces
  joint_names_ = auto_declare<std::vector<std::string>>("joint_names", joint_names_);
  joint_state_interface_types_ =
      auto_declare<std::vector<std::string>>("joint_state_interfaces", joint_state_interface_types_);
  joint_command_interface_types_ =
      auto_declare<std::vector<std::string>>("joint_command_interfaces", joint_command_interface_types_);

  // debug
  debug_ = auto_declare<bool>("debug", false);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BaseManipulatorController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration BaseManipulatorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn BaseManipulatorController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseManipulatorController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseManipulatorController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> BaseManipulatorController::on_export_reference_interfaces()
{
  return {};
}

controller_interface::return_type BaseManipulatorController::update_and_write_commands(const rclcpp::Time& time,
                                                                                          const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type BaseManipulatorController::update_reference_from_subscribers(const rclcpp::Time& time,
                                                                                          const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

}; // namespace manipulator_controllers
