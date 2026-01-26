#ifndef CONSTANT_VELOCITY_PLANNER_HPP_
#define CONSTANT_VELOCITY_PLANNER_HPP_
#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "base_planners/velocity_planner.hpp"

namespace base_planners
{
/*
    ConstantVelocityPlanner implements a velocity planner that maintains constant velocity commands.
*/
class ConstantVelocityPlanner : public VelocityPlanner
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

};

}  // namespace base_planners

#endif  // CONSTANT_VELOCITY_PLANNER_HPP_