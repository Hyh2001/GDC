#include "base_planners/joystick_velocity_planner.hpp"

namespace base_planners
{
controller_interface::CallbackReturn JoystickVelocityPlanner::on_init()
{
    // configure the joystick related stuffs
    int velocity_x_button_ = auto_declare<int>("velocity_x_button", 0);
    int velocity_y_button_ = auto_declare<int>("velocity_y_button", 1);
    int yaw_rate_button_ = auto_declare<int>("yaw_rate_button", 2);
    // configure the ros2 related stuffs
    node_ptr_ = rclcpp::Node::make_shared(std::string(this->get_node()->get_name()) + "_joystick_listener");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
    joy_subscriber_ = this->get_node()->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", qos,
        [this,  velocity_x_button_, velocity_y_button_, yaw_rate_button_](const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            // left stick
            velocity_cmd_raw_[0] = msg->axes[velocity_x_button_]; // vx
            velocity_cmd_raw_[1] = msg->axes[velocity_y_button_]; // vy
            // right stick
            velocity_cmd_raw_[2] = msg->axes[yaw_rate_button_]; // yaw rate
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
    auto ret = VelocityPlanner::on_init();
    if (ret != controller_interface::CallbackReturn::SUCCESS) {
        return ret;
    }
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
    auto ret = VelocityPlanner::update_and_write_commands(time, period);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }
    return controller_interface::return_type::OK;
}

controller_interface::return_type JoystickVelocityPlanner::update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> JoystickVelocityPlanner::on_export_state_interfaces()
{
    return VelocityPlanner::on_export_state_interfaces();
}


}; // base_planners

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(base_planners::JoystickVelocityPlanner, controller_interface::ChainableControllerInterface);
