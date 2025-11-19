#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "base_controllers/multi_joint_pid.hpp"

namespace pendulum_controllers // controller as chained interfaces from estimators
{
    class PendulumPID : public controller_interface::ChainableControllerInterface
    {
    public:
        controller_interface::CallbackReturn on_init() override;
    
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;
        
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & previous_state) override;
    protected:
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
        
        bool on_set_chained_mode(bool chained_mode) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        controller_interface::return_type update_reference_from_subscribers() override;
        
        // pid controller
        double kp_ = 0.0; 
        double kd_ = 0.0;
        double ki_ = 0.0;
        double imax_ = 1e6;
        double imin_ = -1e6;
        base_controllers::MultiJointPID pid_controller_;

        
    };


};  // namespace pendulum_controllers