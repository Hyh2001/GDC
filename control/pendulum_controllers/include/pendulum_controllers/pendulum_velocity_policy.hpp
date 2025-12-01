#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "base_controllers/onnx_policy.hpp"
#include "loggers/logger.hpp"

namespace pendulum_controllers // controller as chained interfaces from estimators
{
    class PendulumVelocityPolicy : public controller_interface::ControllerInterface
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
        
        // interface related
        std::string estimator_name_ = "";
        std::string planner_name_ = "";

        // onnx policy
        std::shared_ptr<base_controllers::OnnxPolicy> velocity_policy_ptr_{nullptr};
        std::vector<double> input_vector_;
        std::vector<double> output_vector_;
        double kp_ = 0.0;
        double kd_ = 0.0;

        // debug related
        bool debug_ = false;
        std::unique_ptr<loggers::Logger> logger_ptr_{nullptr};
    };


};  // namespace pendulum_controllers