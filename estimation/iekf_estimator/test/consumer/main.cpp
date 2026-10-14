#include <type_traits>

#include "iekf_estimator/iekf_estimator.hpp"

class ConsumerEstimator : public iekf_estimator::IEKFEstimator
{
  CallbackReturn on_estimator_configure(const rclcpp_lifecycle::State&) override
  {
    return CallbackReturn::SUCCESS;
  }
};
int main()
{
  static_assert(std::is_abstract_v<iekf_estimator::IEKFEstimator>);
  ConsumerEstimator controller;
  iekf_estimator::inekf::InEKF filter;
  filter.reset();
  return controller.is_chainable() && filter.getState().getPosition().allFinite() ? 0 : 1;
}
