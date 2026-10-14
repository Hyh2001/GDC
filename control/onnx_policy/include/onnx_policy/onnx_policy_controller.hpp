#pragma once

#include "controller_interface/chainable_controller_interface.hpp"
#include "onnx_policy/mdp/action/actions.hpp"
#include "onnx_policy/mdp/observation/observations.hpp"
#include "onnx_policy/onnx_policy.hpp"

namespace onnx_policy
{
// Abstract base: concrete robot packages register their own plugin classes.
class OnnxPolicyController : public controller_interface::ChainableControllerInterface
{
  public:
  using CallbackReturn = controller_interface::CallbackReturn;
  using return_type = controller_interface::return_type;

  CallbackReturn on_init() final;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) final;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) final;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) final;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) final;

  protected:
  virtual CallbackReturn on_policy_init()
  {
    return CallbackReturn::SUCCESS;
  }
  virtual CallbackReturn on_policy_configure(const rclcpp_lifecycle::State& state) = 0;
  virtual CallbackReturn on_policy_activate(const rclcpp_lifecycle::State&)
  {
    return CallbackReturn::SUCCESS;
  }
  virtual CallbackReturn on_policy_deactivate(const rclcpp_lifecycle::State&)
  {
    return CallbackReturn::SUCCESS;
  }
  virtual CallbackReturn on_policy_cleanup(const rclcpp_lifecycle::State&)
  {
    return CallbackReturn::SUCCESS;
  }
  virtual return_type read_policy_inputs(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
  virtual return_type write_policy_commands(const rclcpp::Time& time, const rclcpp::Duration& period,
                                            const std::vector<double>& processed_actions) = 0;

  return_type update_reference_from_subscribers(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    return return_type::OK;
  }
  return_type update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  observation::ObservationManager observation_manager_;
  action::ActionManager action_manager_;
  std::vector<double> observations_;
  std::vector<double> actions_;  // Previous successful raw output; may be seeded on activation.
  std::vector<double> processed_actions_;
  std::unique_ptr<OnnxPolicy> policy_;

  private:
  void reset_policy_state();
  void release_policy();
  return_type fail_update(const char* reason);
  std::vector<double> pending_actions_;
  bool active_{false};
  bool faulted_{true};
};
}  // namespace onnx_policy
