#include "base_planners/velocity_planner.hpp"

namespace base_planners
{
controller_interface::CallbackReturn VelocityPlanner::on_init()
{
    // configure max velocity
    max_velocity_[0] = auto_declare<double>("max_linear_velocity_x", 1.0);
    max_velocity_[1] = auto_declare<double>("max_linear_velocity_y", 1.0);
    max_velocity_[2] = auto_declare<double>("max_angular_velocity_yaw", 0.5);
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration VelocityPlanner::state_interface_configuration() const
{
    // no state interfaces used
    controller_interface::InterfaceConfiguration state_interface_config;
    state_interface_config.type = controller_interface::interface_configuration_type::NONE;
    return state_interface_config; // state_interfaces_
}

controller_interface::InterfaceConfiguration VelocityPlanner::command_interface_configuration() const
{
    // use no command interface
    controller_interface::InterfaceConfiguration command_interface_config;
    command_interface_config.type = controller_interface::interface_configuration_type::NONE;

    return command_interface_config; // command_interfaces_
}

controller_interface::CallbackReturn VelocityPlanner::on_configure(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityPlanner::on_activate(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityPlanner::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VelocityPlanner::update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // clip the velocity commands
    for (size_t i = 0; i < 3; ++i) {
        if (velocity_cmd_[i] > max_velocity_[i]) {
            velocity_cmd_[i] = max_velocity_[i];
        } else if (velocity_cmd_[i] < -max_velocity_[i]) {
            velocity_cmd_[i] = -max_velocity_[i];
        }
    }
    return controller_interface::return_type::OK;
}

controller_interface::return_type VelocityPlanner::update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> VelocityPlanner::on_export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    std::string planner_name = this->get_name();
    state_interfaces.emplace_back(hardware_interface::StateInterface(
            planner_name, "lin_x_vel_ref", &velocity_cmd_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
            planner_name, "lin_y_vel_ref", &velocity_cmd_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
            planner_name, "yaw_rate_ref", &velocity_cmd_[2]));
    return state_interfaces;
}


}; // base_planners
