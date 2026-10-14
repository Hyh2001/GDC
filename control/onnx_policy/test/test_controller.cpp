#include <gtest/gtest.h>

#include <limits>

#include "onnx_policy/onnx_policy_controller.hpp"

using namespace onnx_policy;
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

namespace
{
class TestController : public OnnxPolicyController
{
  public:
  std::vector<std::string> calls;
  Return read_result{Return::OK}, write_result{Return::OK};
  CallbackReturn configure_result{CallbackReturn::SUCCESS}, activate_result{CallbackReturn::SUCCESS};
  bool empty_terms{false}, throw_read{false};
  std::vector<double> seed{0, 0};
  int reads{0}, writes{0};

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {"joint/position"}};
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {"joint/position", "joint2/position"}};
  }

  const auto& previous_actions() const
  {
    return actions_;
  }
  const auto& observations() const
  {
    return observations_;
  }
  bool has_policy() const
  {
    return static_cast<bool>(policy_);
  }
  int observation_size() const
  {
    return observation_manager_.total_observation_dim();
  }

  protected:
  CallbackReturn on_policy_init() override
  {
    calls.push_back("init");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_policy_configure(const rclcpp_lifecycle::State&) override
  {
    calls.push_back("configure");
    reference_interfaces_.assign(1, 0.0);
    if (empty_terms)
      return configure_result;
    observation::ObservationTermCfg cfg;
    cfg.observation_dim = 1;
    observation_manager_.add_term(
        std::make_unique<observation::ObservationTerm>(cfg, [this] { return std::vector<double>{input_}; }));
    observation_manager_.add_term(std::make_unique<observation::ObservationTerm>(
        cfg, [this] { return std::vector<double>{reference_interfaces_[0]}; }));
    cfg.observation_dim = 2;
    observation_manager_.add_term(std::make_unique<observation::ObservationTerm>(cfg, [this] { return actions_; }));
    action_manager_.add_term(
        std::make_unique<action::JointPositionActionTerm>(action::ActionTermCfg{"", 2, {2, 3}, {10, 20}}));
    return configure_result;
  }
  CallbackReturn on_policy_activate(const rclcpp_lifecycle::State&) override
  {
    calls.push_back("activate");
    if (state_interfaces_.size() != 1 || command_interfaces_.size() != 2 ||
        state_interfaces_[0].get_name() != "joint/position" || command_interfaces_[0].get_name() != "joint/position" ||
        command_interfaces_[1].get_name() != "joint2/position")
      return CallbackReturn::ERROR;
    reference_interfaces_[0] = 0;
    actions_ = seed;
    return activate_result;
  }
  CallbackReturn on_policy_deactivate(const rclcpp_lifecycle::State&) override
  {
    calls.push_back("deactivate");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_policy_cleanup(const rclcpp_lifecycle::State&) override
  {
    calls.push_back(policy_ ? "cleanup_with_policy" : "cleanup_without_policy");
    return CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back(get_name(), "target", &reference_interfaces_[0]);
    return interfaces;
  }
  Return read_policy_inputs(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    calls.push_back("read");
    ++reads;
    if (throw_read)
      throw std::runtime_error("test read failure");
    const auto value = state_interfaces_[0].get_optional();
    if (!value)
      return Return::ERROR;
    input_ = *value;
    return read_result;
  }
  Return write_policy_commands(const rclcpp::Time&, const rclcpp::Duration&,
                               const std::vector<double>& actions) override
  {
    calls.push_back("write");
    ++writes;
    if (write_result != Return::OK)
      return write_result;
    for (size_t i = 0; i < actions.size(); ++i)
      if (!command_interfaces_[i].set_value(actions[i]))
        return Return::ERROR;
    return Return::OK;
  }

  private:
  double input_{0};
};

struct Hardware
{
  hardware_interface::StateInterface::SharedPtr state =
      std::make_shared<hardware_interface::StateInterface>("joint", "position");
  hardware_interface::CommandInterface::SharedPtr first =
      std::make_shared<hardware_interface::CommandInterface>("joint", "position");
  hardware_interface::CommandInterface::SharedPtr second =
      std::make_shared<hardware_interface::CommandInterface>("joint2", "position");
  void assign(TestController& controller)
  {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(state);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(first, nullptr);
    commands.emplace_back(second, nullptr);
    controller.assign_interfaces(std::move(commands), std::move(states));
    (void)state->set_value(1.0);
    (void)first->set_value(-100.0);
    (void)second->set_value(-200.0);
  }
};

void initialize(TestController& controller, const std::string& name = "policy", bool recurrent = false)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.controller_manager_update_rate = 100;
  params.update_rate = 100;
  params.node_options.parameter_overrides(
      {rclcpp::Parameter("model.path",
                         std::string(FIXTURE_DIR) + (recurrent ? "/controller_rnn.onnx" : "/controller.onnx")),
       rclcpp::Parameter("model.type", recurrent ? "rnn" : "mlp"), rclcpp::Parameter("model.hidden_layers", 1),
       rclcpp::Parameter("model.hidden_dim", 2)});
  ASSERT_EQ(controller.init(params), Return::OK);
}

Return step(TestController& controller)
{
  return controller.update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(.01));
}

class ControllerTest : public ::testing::Test
{
  protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};
}  // namespace

TEST_F(ControllerTest, LifecycleStateReferencesAndPreviousActions)
{
  TestController controller;
  initialize(controller);
  ASSERT_EQ(controller.on_configure({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.observation_size(), 4);
  Hardware hardware;
  hardware.assign(controller);
  auto references = controller.export_reference_interfaces();
  ASSERT_EQ(references.size(), 1u);
  EXPECT_EQ(references[0]->get_name(), "policy/target");
  ASSERT_EQ(controller.on_activate({}), CallbackReturn::SUCCESS);
  (void)references[0]->set_value(2.0);
  ASSERT_EQ(step(controller), Return::OK);
  EXPECT_EQ(controller.observations(), (std::vector<double>{1, 2, 0, 0}));
  EXPECT_EQ(controller.previous_actions(), (std::vector<double>{1, 2}));
  EXPECT_DOUBLE_EQ(hardware.first->get_optional<double>().value(), 12);
  EXPECT_DOUBLE_EQ(hardware.second->get_optional<double>().value(), 26);
  ASSERT_EQ(step(controller), Return::OK);
  EXPECT_EQ(controller.previous_actions(), (std::vector<double>{2, 4}));
  EXPECT_EQ(controller.calls,
            (std::vector<std::string>{"init", "configure", "activate", "read", "write", "read", "write"}));
  ASSERT_EQ(controller.on_deactivate({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(step(controller), Return::ERROR);
  EXPECT_EQ(controller.previous_actions(), (std::vector<double>{0, 0}));
  EXPECT_DOUBLE_EQ(hardware.first->get_optional<double>().value(), 14);
  ASSERT_EQ(controller.on_cleanup({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.calls.back(), "cleanup_with_policy");
  EXPECT_FALSE(controller.has_policy());
  ASSERT_EQ(controller.on_configure({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.observation_size(), 4);
}

TEST_F(ControllerTest, FailureLatchesAndNeverCommitsFailedActions)
{
  TestController controller;
  initialize(controller);
  ASSERT_EQ(controller.on_configure({}), CallbackReturn::SUCCESS);
  Hardware hardware;
  hardware.assign(controller);
  ASSERT_EQ(controller.on_activate({}), CallbackReturn::SUCCESS);
  controller.read_result = Return::ERROR;
  EXPECT_EQ(step(controller), Return::ERROR);
  EXPECT_EQ(controller.writes, 0);
  EXPECT_DOUBLE_EQ(hardware.first->get_optional<double>().value(), -100);
  controller.read_result = Return::OK;
  EXPECT_EQ(step(controller), Return::ERROR);
  EXPECT_EQ(controller.reads, 1);
  ASSERT_EQ(controller.on_deactivate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate({}), CallbackReturn::SUCCESS);
  controller.write_result = Return::ERROR;
  EXPECT_EQ(step(controller), Return::ERROR);
  EXPECT_EQ(controller.previous_actions(), (std::vector<double>{0, 0}));
  EXPECT_DOUBLE_EQ(hardware.first->get_optional<double>().value(), -100);
  EXPECT_EQ(step(controller), Return::ERROR);
}

TEST_F(ControllerTest, NonfiniteInputsAndExceptionsShortCircuit)
{
  for (bool throw_read : {false, true})
  {
    TestController controller;
    initialize(controller);
    ASSERT_EQ(controller.on_configure({}), CallbackReturn::SUCCESS);
    Hardware hardware;
    hardware.assign(controller);
    ASSERT_EQ(controller.on_activate({}), CallbackReturn::SUCCESS);
    controller.throw_read = throw_read;
    (void)hardware.state->set_value(std::numeric_limits<double>::quiet_NaN());
    EXPECT_EQ(step(controller), Return::ERROR);
    EXPECT_EQ(controller.writes, 0);
    EXPECT_DOUBLE_EQ(hardware.first->get_optional<double>().value(), -100);
  }
}

TEST_F(ControllerTest, RecurrentWarmupResetAndIndependentInstances)
{
  TestController first, second;
  initialize(first, "first", true);
  initialize(second, "second", true);
  ASSERT_EQ(first.on_configure({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_configure({}), CallbackReturn::SUCCESS);
  Hardware hardware1, hardware2;
  hardware1.assign(first);
  hardware2.assign(second);
  ASSERT_EQ(first.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(step(first), Return::OK);
  EXPECT_EQ(first.previous_actions(), (std::vector<double>{1, 0}));
  ASSERT_EQ(step(first), Return::OK);
  EXPECT_EQ(first.previous_actions(), (std::vector<double>{4, 1}));
  EXPECT_EQ(second.previous_actions(), (std::vector<double>{0, 0}));
  ASSERT_EQ(step(second), Return::OK);
  EXPECT_EQ(second.previous_actions(), (std::vector<double>{1, 0}));
  ASSERT_EQ(first.on_deactivate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(first.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(step(first), Return::OK);
  EXPECT_EQ(first.previous_actions(), (std::vector<double>{1, 0}));
}

TEST_F(ControllerTest, ActivationCanSeedPreviousActions)
{
  TestController controller;
  initialize(controller);
  ASSERT_EQ(controller.on_configure({}), CallbackReturn::SUCCESS);
  Hardware hardware;
  hardware.assign(controller);
  controller.seed = {5, 6};
  ASSERT_EQ(controller.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(step(controller), Return::OK);
  EXPECT_EQ(controller.observations(), (std::vector<double>{1, 0, 5, 6}));
  EXPECT_EQ(controller.previous_actions(), (std::vector<double>{6, 6}));
}

TEST_F(ControllerTest, ConfigurationAndActivationFailures)
{
  TestController controller;
  initialize(controller);
  EXPECT_EQ(controller.on_activate({}), CallbackReturn::ERROR);
  controller.empty_terms = true;
  EXPECT_EQ(controller.on_configure({}), CallbackReturn::ERROR);
  EXPECT_FALSE(controller.has_policy());
  controller.empty_terms = false;
  controller.configure_result = CallbackReturn::FAILURE;
  EXPECT_EQ(controller.on_configure({}), CallbackReturn::FAILURE);
  EXPECT_FALSE(controller.has_policy());
  controller.configure_result = CallbackReturn::SUCCESS;
  controller.get_node()->set_parameter(rclcpp::Parameter("model.type", "invalid"));
  EXPECT_EQ(controller.on_configure({}), CallbackReturn::ERROR);
  controller.get_node()->set_parameter(rclcpp::Parameter("model.type", "mlp"));
  controller.get_node()->set_parameter(rclcpp::Parameter("model.path", "relative.onnx"));
  EXPECT_EQ(controller.on_configure({}), CallbackReturn::ERROR);
  controller.get_node()->set_parameter(rclcpp::Parameter("model.path", std::string(FIXTURE_DIR) + "/controller.onnx"));
  ASSERT_EQ(controller.on_configure({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.on_activate({}), CallbackReturn::ERROR);  // No loaned interfaces.
  Hardware hardware;
  hardware.assign(controller);
  controller.seed = {1};
  EXPECT_EQ(controller.on_activate({}), CallbackReturn::ERROR);
  EXPECT_EQ(step(controller), Return::ERROR);
}
