#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "pendulum_estimators/base_pendulum_estimator.hpp"

#include "pendulum_msgs/msg/pendulum_est.hpp"

namespace pendulum_estimators // estimator as a chainable controller
{
    class DummyEstimator : public BasePendulumEstimator
    {
    public:
        controller_interface::CallbackReturn on_init() override;
    
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

    protected:
        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
        controller_interface::return_type update_reference_from_subscribers() override;

        std::string node_name_ = "";
        std::string subscribe_topic_name_ = "";
        rclcpp::Node::SharedPtr node_ptr_ = nullptr;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Subscription<pendulum_msgs::msg::PendulumEst>::SharedPtr PendulumEst_subscriber_ = nullptr;

    };


};  // namespace pendulum_estimators