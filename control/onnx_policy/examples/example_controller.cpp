// Concrete packages may register this class with pluginlib; this package does not.
#include "onnx_policy/onnx_policy_controller.hpp"

namespace example
{
class JointPolicyController : public onnx_policy::OnnxPolicyController
{
  public:
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {joint_ + "/position"}};
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {joint_ + "/position"}};
  }

  protected:
  CallbackReturn on_policy_init() override
  {
    joint_ = auto_declare<std::string>("joint_name", "joint");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_policy_configure(const rclcpp_lifecycle::State&) override
  {
    reference_interfaces_.assign(1, 0.0);
    onnx_policy::observation::ObservationTermCfg cfg;
    cfg.name = "position_and_target";
    cfg.observation_dim = 2;
    observation_manager_.add_term(std::make_unique<onnx_policy::observation::ObservationTerm>(
        cfg,
        [this] {
          return std::vector<double>{position_, reference_interfaces_[0]};
        }));
    action_manager_.add_term(std::make_unique<onnx_policy::action::JointPositionActionTerm>(
        onnx_policy::action::ActionTermCfg{"joint_position", 1, {0.25}, {0.0}}));
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_policy_activate(const rclcpp_lifecycle::State&) override
  {
    if (state_interfaces_.size() != 1 || command_interfaces_.size() != 1 ||
        state_interfaces_[0].get_name() != joint_ + "/position" ||
        command_interfaces_[0].get_name() != joint_ + "/position")
      return CallbackReturn::ERROR;
    reference_interfaces_[0] = 0.0;
    return CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back(get_name(), "target", &reference_interfaces_[0]);
    return interfaces;
  }
  return_type read_policy_inputs(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    const auto value = state_interfaces_[0].get_optional();
    if (!value)
      return return_type::ERROR;
    position_ = *value;
    return return_type::OK;
  }
  return_type write_policy_commands(const rclcpp::Time&, const rclcpp::Duration&,
                                    const std::vector<double>& actions) override
  {
    return command_interfaces_[0].set_value(actions[0]) ? return_type::OK : return_type::ERROR;
  }

  private:
  std::string joint_;
  double position_{0};
};
}  // namespace example
