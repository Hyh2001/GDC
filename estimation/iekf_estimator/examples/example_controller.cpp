#include "iekf_estimator/iekf_estimator.hpp"

namespace example
{
// Register this concrete class with pluginlib in your robot's controller package.
class RobotEstimator : public iekf_estimator::IEKFEstimator
{
  protected:
  CallbackReturn on_estimator_configure(const rclcpp_lifecycle::State&) override
  {
    // Common model, parameters, estimator and input buffers are already initialized.
    // Allocate any robot-specific subscriber handoff buffers here.
    return CallbackReturn::SUCCESS;
  }
  return_type prepare_estimator_inputs(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    // Optionally apply a robot-specific orientation alignment from a safe subscriber handoff.
    // inputs_->quaternion_ is world-from-body; joint buffers are in compact model order.
    return return_type::OK;
  }
};
}  // namespace example
