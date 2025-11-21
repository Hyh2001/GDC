#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "control_toolbox/sinusoid.hpp"

namespace pendulum_planners
{
    class PendulumJointPeriodicPlanner : public controller_interface::ControllerInterface
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
        controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        std::string ref_controller_name_ = "";
        double offset_ = 0.0;
        double amplitude_ = 1.0;
        double frequency_ = 0.5; // Hz
        double phase_ = 0.0; // rad
        std::shared_ptr<control_toolbox::Sinusoid> sinusoid_planner_ptr_ = nullptr;
        double q_;
        double qd_; 
        double qdd_;
    };

}; // namespace pendulum_planners