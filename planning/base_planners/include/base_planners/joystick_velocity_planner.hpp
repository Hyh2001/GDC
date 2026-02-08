#ifndef JOYSTICK_VELOCITY_PLANNER_HPP_
#define JOYSTICK_VELOCITY_PLANNER_HPP_
#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "Eigen/Dense"

#include "controller_interface/chainable_controller_interface.hpp"
#include "control_toolbox/low_pass_filter.hpp"

#include "base_planners/velocity_planner.hpp"

namespace base_planners
{
/*
    JoystickVelocityPlanner implements a velocity planner taking velocity commands from joystick signals and
    applying a low-pass filter to smooth the commands.
*/

class JoystickVelocityPlanner : public VelocityPlanner
{
public:
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

protected:
    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

    int velocity_x_button_{0};
    int velocity_y_button_{1};
    int yaw_rate_button_{2};
    double sampling_frequency_{50.0}; // Hz
    double damping_frequency_{1.0}; // Hz
    double damping_intensity_{0.0}; // dB

    std::array<double, 3> velocity_cmd_raw_{0.0, 0.0, 0.0}; // vx, vy, yaw rate
    rclcpp::Node::SharedPtr node_ptr_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_ = nullptr;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::array<std::shared_ptr<control_toolbox::LowPassFilter<double>>, 3> lp_filters_;
};

}; // base_planners

#endif // JOYSTICK_VELOCITY_PLANNER_HPP_
