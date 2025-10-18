#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "base_quadruped_estimators/base_quadruped_estimator.hpp"

#include "quadruped_msgs/msg/quad_est.hpp"

namespace quadruped_controllers // estimator as a chainable controller
{
    class DummyEstimator : public quadruped_controllers::BaseQuadrupedEstimator
    {
    public:
        controller_interface::CallbackReturn on_init() override;
    
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

    protected:
        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
        std::string node_name_ = "";
        std::string subscribe_topic_name_ = "";
        rclcpp::Node::SharedPtr node_ptr_ = nullptr;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Subscription<quadruped_msgs::msg::QuadEst>::SharedPtr QuadEst_subscriber_ = nullptr;

    };


}; 