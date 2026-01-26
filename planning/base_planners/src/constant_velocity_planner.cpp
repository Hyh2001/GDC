#include "base_planners/constant_velocity_planner.hpp"

namespace base_planners 
{
controller_interface::CallbackReturn ConstantVelocityPlanner::on_init()
{
    // configure target velocity and ignore max velocity
    velocity_cmd_[0] = auto_declare<double>("vel_cmd_x", 0.0);
    velocity_cmd_[1] = auto_declare<double>("vel_cmd_y", 0.0);
    velocity_cmd_[2] = auto_declare<double>("yaw_rate_cmd", 0.0); 

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ConstantVelocityPlanner::state_interface_configuration() const
{
    // no state interfaces used
    controller_interface::InterfaceConfiguration state_interface_config;
    state_interface_config.type = controller_interface::interface_configuration_type::NONE;
    return state_interface_config; // state_interfaces_
}

controller_interface::InterfaceConfiguration ConstantVelocityPlanner::command_interface_configuration() const
{
    // use no command interface
    controller_interface::InterfaceConfiguration command_interface_config;
    command_interface_config.type = controller_interface::interface_configuration_type::NONE;

    return command_interface_config; // command_interfaces_  
}

controller_interface::CallbackReturn ConstantVelocityPlanner::on_configure(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConstantVelocityPlanner::on_activate(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConstantVelocityPlanner::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ConstantVelocityPlanner::update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // update the message
    return controller_interface::return_type::OK;
}

controller_interface::return_type ConstantVelocityPlanner::update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ConstantVelocityPlanner::on_export_state_interfaces()
{
    return VelocityPlanner::on_export_state_interfaces();
}


}; // base_planners

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(base_planners::ConstantVelocityPlanner, controller_interface::ChainableControllerInterface);