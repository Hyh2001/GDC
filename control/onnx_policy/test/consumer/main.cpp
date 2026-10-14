#include <type_traits>

#include "onnx_policy/onnx_policy_controller.hpp"

class ConsumerController : public onnx_policy::OnnxPolicyController
{
  public:
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  protected:
  CallbackReturn on_policy_configure(const rclcpp_lifecycle::State&) override
  {
    return CallbackReturn::SUCCESS;
  }
  return_type read_policy_inputs(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    return return_type::OK;
  }
  return_type write_policy_commands(const rclcpp::Time&, const rclcpp::Duration&, const std::vector<double>&) override
  {
    return return_type::OK;
  }
};

int main(int argc, char** argv)
{
  static_assert(std::is_abstract_v<onnx_policy::OnnxPolicyController>);
  if (argc != 2)
    return 1;
  ConsumerController controller;
  onnx_policy::OnnxPolicy policy(argv[1], 3, 2);
  return policy.infer({1, 2, 3}) == std::vector<double>{3, 3} ? 0 : 1;
}
