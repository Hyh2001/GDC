#include "base_planners/joystick_velocity_planner.hpp"

namespace base_planners 
{
controller_interface::CallbackReturn JoystickVelocityPlanner::on_init()
{
    // configure the ros2 related stuffs
    node_ptr_ = rclcpp::Node::make_shared(std::string(this->get_node()->get_name()) + "_joystick_listener");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
    joy_subscriber_ = this->get_node()->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", qos,
        [this](const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            // left stick
            velocity_cmd_raw_[0] = msg->axes[1]; // vx
            velocity_cmd_raw_[1] = msg->axes[0]; // vy
            // right stick
            velocity_cmd_raw_[2] = msg->axes[3]; // yaw rate
        });
    executor_.add_node(node_ptr_);
    // configure the filter
    sampling_frequency_ = auto_declare<double>("sampling_frequency", 50.0);
    damping_frequency_ = auto_declare<double>("damping_frequency", 1.0);
    damping_intensity_ = auto_declare<double>("damping_intensity", 0.0);
    for (size_t i = 0; i < 3; ++i) {
        lp_filters_[i] = std::make_shared<control_toolbox::LowPassFilter<double>>(
            sampling_frequency_, damping_frequency_, damping_intensity_);
        lp_filters_[i]->configure();
    }
    // configure max velocity
    max_velocity_[0] = auto_declare<double>("max_linear_velocity_x", 1.0);
    max_velocity_[1] = auto_declare<double>("max_linear_velocity_y", 1.0);
    max_velocity_[2] = auto_declare<double>("max_angular_velocity_yaw", 0.5);
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration JoystickVelocityPlanner::state_interface_configuration() const
{
    // no state interfaces used
    controller_interface::InterfaceConfiguration state_interface_config;
    state_interface_config.type = controller_interface::interface_configuration_type::NONE;
    return state_interface_config; // state_interfaces_
}

controller_interface::InterfaceConfiguration JoystickVelocityPlanner::command_interface_configuration() const
{
    // use no command interface
    controller_interface::InterfaceConfiguration command_interface_config;
    command_interface_config.type = controller_interface::interface_configuration_type::NONE;

    return command_interface_config; // command_interfaces_  
}

controller_interface::CallbackReturn JoystickVelocityPlanner::on_configure(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickVelocityPlanner::on_activate(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickVelocityPlanner::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JoystickVelocityPlanner::update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // update the message
    executor_.spin_some(std::chrono::milliseconds(1));
    // filter the velocity commands
    for (size_t i = 0; i < 3; ++i) {
        lp_filters_[i]->update(velocity_cmd_raw_[i], velocity_cmd_[i]);
    }
    // clip the velocity commands
    for (size_t i = 0; i < 3; ++i) {
        if (velocity_cmd_[i] > max_velocity_[i]) {
            velocity_cmd_[i] = max_velocity_[i];
        } else if (velocity_cmd_[i] < -max_velocity_[i]) {  
            velocity_cmd_[i] = -max_velocity_[i];
        }
    }
    // std::cout << "Velocity Command (vx, vy, yaw_rate): "
    //         << velocity_cmd_[0] << ", "
    //         << velocity_cmd_[1] << ", "
    //         << velocity_cmd_[2] << std::endl;
    return controller_interface::return_type::OK;
}

controller_interface::return_type JoystickVelocityPlanner::update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> JoystickVelocityPlanner::on_export_state_interfaces()
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

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(base_planners::JoystickVelocityPlanner, controller_interface::ChainableControllerInterface);