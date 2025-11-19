#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"

#include "pendulum_msgs/msg/pendulum_est.hpp"

namespace pendulum_estimators // estimator as a chainable controller
{
    class DummyEstimator : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::CallbackReturn on_init() override;
    
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

    protected:
        controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        
        // sensor readings
        std::string ref_controller_name_ = "";
        double joint_pos_ = 0.0;
        double joint_vel_ = 0.0;
        double joint_tau_ = 0.0;
        std::array<double, 2> tip_pos_ = {0.0, 0.0}; // y, z
        std::array<double, 2> tip_vel_ = {0.0, 0.0}; // vy, vz


        // ros2 related
        std::string node_name_ = "";
        std::string subscribe_topic_name_ = "";
        rclcpp::Node::SharedPtr node_ptr_ = nullptr;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Subscription<pendulum_msgs::msg::PendulumEst>::SharedPtr PendulumEst_subscriber_ = nullptr;

    };


};  // namespace pendulum_estimators