#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>

#include "iekf_estimator/iekf_estimator.hpp"

namespace
{
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;
class TestEstimator : public iekf_estimator::IEKFEstimator
{
  public:
  std::vector<std::string> calls;
  Return input_result{Return::OK};
  CallbackReturn configure_result{CallbackReturn::SUCCESS}, activate_result{CallbackReturn::SUCCESS};
  bool throw_input{false}, resize_input{false};
  const auto& inputs() const
  {
    return *inputs_;
  }
  const auto& core() const
  {
    return *estimator_;
  }
  const auto& model() const
  {
    return model_;
  }
  const auto& parameters() const
  {
    return *parameters_;
  }
  bool has_estimator() const
  {
    return bool(estimator_);
  }
  std::vector<hardware_interface::StateInterface> raw_exports()
  {
    return on_export_state_interfaces();
  }

  protected:
  CallbackReturn on_estimator_init() override
  {
    calls.push_back("init");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_estimator_configure(const rclcpp_lifecycle::State&) override
  {
    calls.push_back(estimator_ && inputs_ && data_ ? "configure_ready" : "configure_empty");
    return configure_result;
  }
  CallbackReturn on_estimator_activate(const rclcpp_lifecycle::State&) override
  {
    calls.push_back(estimator_->isinit ? "activate_initialized" : "activate_reset");
    return activate_result;
  }
  CallbackReturn on_estimator_deactivate(const rclcpp_lifecycle::State&) override
  {
    calls.push_back("deactivate");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_estimator_cleanup(const rclcpp_lifecycle::State&) override
  {
    calls.push_back(estimator_ ? "cleanup_ready" : "cleanup_empty");
    return CallbackReturn::SUCCESS;
  }
  Return prepare_estimator_inputs(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    calls.push_back("prepare");
    if (throw_input)
    {
      throw std::runtime_error("input failure");
    }
    if (resize_input)
    {
      inputs_->joint_states_position_.resize(1);
    }
    return input_result;
  }
};

std::vector<rclcpp::Parameter> configuration(bool momentum = false)
{
  std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("model.path", std::string(FIXTURE_DIR) + "/robot.urdf"),
      rclcpp::Parameter("joint_names", std::vector<std::string>{"right_joint", "arm_joint", "left_joint"}),
      rclcpp::Parameter("contact_names", std::vector<std::string>{"right_foot", "left_foot"}),
      rclcpp::Parameter("interfaces.joint_names", std::vector<std::string>{"robot/right", "robot/arm", "robot/left"}),
      rclcpp::Parameter("interfaces.imu", "robot/imu"),
      rclcpp::Parameter("joint_position_std", std::vector<double>{.01, .02, .03}),
      rclcpp::Parameter("using_mob", momentum)};
  if (!momentum)
  {
    parameters.emplace_back("interfaces.contacts", std::vector<std::string>{"right_sensor/state", "left_sensor/state"});
  }
  return parameters;
}
void initialize(TestEstimator& estimator, const std::string& name = "estimator",
                std::vector<rclcpp::Parameter> overrides = configuration(), bool description = false)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.controller_manager_update_rate = 100;
  params.update_rate = 100;
  if (description)
  {
    std::ifstream file(std::string(FIXTURE_DIR) + "/robot.urdf");
    std::ostringstream text;
    text << file.rdbuf();
    params.robot_description = text.str();
  }
  params.node_options.parameter_overrides(overrides);
  ASSERT_EQ(estimator.init(params), Return::OK);
}
struct Hardware
{
  std::map<std::string, hardware_interface::StateInterface::SharedPtr> states;
  void assign(TestEstimator& estimator, bool missing = false, bool duplicate = false)
  {
    std::vector<hardware_interface::LoanedStateInterface> loans;
    auto names = estimator.state_interface_configuration().names;
    std::reverse(names.begin(), names.end());
    for (const auto& name : names)
    {
      const auto slash = name.rfind('/');
      auto state = std::make_shared<hardware_interface::StateInterface>(name.substr(0, slash), name.substr(slash + 1));
      states.emplace(name, state);
      loans.emplace_back(state);
      ASSERT_TRUE(state->set_value(0.0));
    }
    if (missing)
    {
      loans.pop_back();
    }
    if (duplicate)
    {
      loans.pop_back();
      loans.emplace_back(states.at(names[0]));
    }
    estimator.assign_interfaces({}, std::move(loans));
    set("robot/imu/orientation.w", 1.0);
    set("robot/imu/linear_acceleration.z", 9.81);
    set("robot/right/position", -.2);
    set("robot/left/position", .3);
    set("robot/arm/position", .7);
    if (states.count("right_sensor/state"))
    {
      set("right_sensor/state", 1.0);
      set("left_sensor/state", 0.0);
    }
  }
  void set(const std::string& name, double value)
  {
    ASSERT_TRUE(states.at(name)->set_value(value));
  }
};
Return step(TestEstimator& estimator, double time = 1.0, double period = .01)
{
  return estimator.update(rclcpp::Time(static_cast<int64_t>(time * 1e9), RCL_ROS_TIME),
                          rclcpp::Duration::from_seconds(period));
}
using Exports = std::vector<hardware_interface::StateInterface::ConstSharedPtr>;
double value(const Exports& exports, const std::string& name)
{
  for (const auto& state : exports)
  {
    if (state->get_name() == name)
    {
      return state->get_optional<double>().value();
    }
  }
  ADD_FAILURE() << "Missing export: " << name;
  return std::numeric_limits<double>::quiet_NaN();
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

TEST_F(ControllerTest, NamedInterfacesLifecycleAndExportedState)
{
  TestEstimator estimator;
  initialize(estimator);
  ASSERT_EQ(estimator.on_configure({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(estimator.parameters().rate_, 100);
  EXPECT_EQ(estimator.command_interface_configuration().type, controller_interface::interface_configuration_type::NONE);
  EXPECT_TRUE(estimator.export_reference_interfaces().empty());
  EXPECT_TRUE(estimator.is_chainable());
  auto exports = estimator.export_state_interfaces();
  ASSERT_EQ(exports.size(), 29u);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/valid"), 0.0);
  EXPECT_TRUE(std::isnan(value(exports, "estimator/base/position.x")));
  Hardware hardware;
  hardware.assign(estimator);
  ASSERT_EQ(estimator.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(step(estimator), Return::OK);
  EXPECT_EQ(estimator.calls, (std::vector<std::string>{"init", "configure_ready", "activate_reset", "prepare"}));
  EXPECT_TRUE(estimator.core().isinit);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/valid"), 1.0);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/sample_time"), 1.0);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/joints/right_joint/position"), -.2);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/joints/left_joint/position"), .3);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/joints/arm_joint/position"), .7);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/contacts/right_foot/state"), 1.0);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/contacts/left_foot/state"), 0.0);
  for (const auto& state : exports)
  {
    EXPECT_TRUE(std::isfinite(state->get_optional<double>().value()));
  }
  const auto id = estimator.model().getJointId("right_joint");
  EXPECT_DOUBLE_EQ(estimator.inputs().joint_states_position_[estimator.model().joints[id].idx_q() - 7], -.2);
  ASSERT_EQ(step(estimator, 1.01), Return::OK);
  ASSERT_EQ(estimator.on_deactivate({}), CallbackReturn::SUCCESS);
  EXPECT_FALSE(estimator.core().isinit);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/valid"), 0.0);
  EXPECT_EQ(step(estimator), Return::ERROR);
  ASSERT_EQ(estimator.on_cleanup({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(estimator.calls.back(), "cleanup_ready");
  EXPECT_FALSE(estimator.has_estimator());
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/valid"), 0.0);
}

TEST_F(ControllerTest, ExtrinsicsAndOutputFrames)
{
  auto params = configuration();
  const Eigen::Quaterniond q_bi(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond q_wb(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
  params.emplace_back("q_body_imu", std::vector<double>{q_bi.w(), q_bi.x(), q_bi.y(), q_bi.z()});
  params.emplace_back("p_body_imu", std::vector<double>{.1, -.2, .3});
  params.emplace_back("ground_height_offset", 0.0);
  params.emplace_back("using_lo_p", false);
  params.emplace_back("using_lo_v", false);
  TestEstimator estimator;
  initialize(estimator, "frames", params);
  ASSERT_EQ(estimator.on_configure({}), CallbackReturn::SUCCESS);
  Hardware hardware;
  hardware.assign(estimator);
  const auto q_wi = q_wb * q_bi;
  EXPECT_DOUBLE_EQ(estimator.parameters().ground_height_offset_, 0.0);
  for (const auto& pair : std::vector<std::pair<std::string, double>>{{"orientation.w", q_wi.w()},
                                                                      {"orientation.x", q_wi.x()},
                                                                      {"orientation.y", q_wi.y()},
                                                                      {"orientation.z", q_wi.z()}})
  {
    hardware.set("robot/imu/" + pair.first, pair.second);
  }
  const Eigen::Vector3d omega(.1, -.2, .3);
  const Eigen::Vector3d acceleration(.2, -.3, 9.81);
  const Eigen::Vector3d measured_omega = q_bi.conjugate() * omega;
  const Eigen::Vector3d measured_acceleration = q_bi.conjugate() * acceleration;
  for (size_t i = 0; i < 3; ++i)
  {
    const auto suffix = std::string(1, "xyz"[i]);
    hardware.set("robot/imu/angular_velocity." + suffix, measured_omega[i]);
    hardware.set("robot/imu/linear_acceleration." + suffix, measured_acceleration[i]);
  }
  const auto exports = estimator.export_state_interfaces();
  ASSERT_EQ(estimator.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(step(estimator), Return::OK);
  EXPECT_TRUE(estimator.inputs().omega_b_.isApprox(omega, 1e-12));
  EXPECT_TRUE(estimator.inputs().accel_b_.isApprox(acceleration, 1e-12));
  EXPECT_TRUE(estimator.inputs().quaternion_.toRotationMatrix().isApprox(q_wb.toRotationMatrix(), 1e-12));
  const auto& x = estimator.core().x_est_;
  const Eigen::Quaterniond q(x[6], x[7], x[8], x[9]);
  const Eigen::Vector3d expected_position = x.head<3>() - q * Eigen::Vector3d(.1, -.2, .3);
  for (size_t i = 0; i < 3; ++i)
  {
    const auto suffix = std::string(1, "xyz"[i]);
    EXPECT_NEAR(value(exports, "frames/base/position." + suffix), expected_position[i], 1e-12);
    EXPECT_NEAR(value(exports, "frames/base/linear_velocity." + suffix), estimator.core().v_b_est_[i], 1e-12);
    EXPECT_NEAR(value(exports, "frames/base/angular_velocity." + suffix), omega[i] - x[10 + i], 1e-12);
  }
}

TEST_F(ControllerTest, FailureLatchesAndReactivationResetsWithoutMovingExports)
{
  TestEstimator estimator;
  initialize(estimator);
  ASSERT_EQ(estimator.on_configure({}), CallbackReturn::SUCCESS);
  Hardware hardware;
  hardware.assign(estimator);
  const auto exports = estimator.export_state_interfaces();
  ASSERT_EQ(estimator.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(step(estimator), Return::OK);
  const double initial_position = value(exports, "estimator/base/position.z");
  const auto initial_state = estimator.core().x_est_;
  estimator.input_result = Return::ERROR;
  EXPECT_EQ(step(estimator, 2.0), Return::ERROR);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/valid"), 0.0);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/sample_time"), 1.0);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/base/position.z"), initial_position);
  EXPECT_TRUE(estimator.core().x_est_.isApprox(initial_state, 0));
  const auto calls = estimator.calls.size();
  estimator.input_result = Return::OK;
  EXPECT_EQ(step(estimator), Return::ERROR);
  EXPECT_EQ(estimator.calls.size(), calls);
  ASSERT_EQ(estimator.on_deactivate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(estimator.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(step(estimator), Return::OK);
  EXPECT_DOUBLE_EQ(value(exports, "estimator/status/valid"), 1.0);
  EXPECT_TRUE(estimator.core().x_est_.isApprox(initial_state, 1e-12));
  hardware.set("robot/left/position", std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(step(estimator), Return::ERROR);
  EXPECT_TRUE(estimator.core().x_est_.isApprox(initial_state, 0));
}

TEST_F(ControllerTest, RejectsInvalidRuntimeDataAndHookExceptions)
{
  for (int failure = 0; failure < 5; ++failure)
  {
    TestEstimator estimator;
    initialize(estimator, "invalid" + std::to_string(failure));
    ASSERT_EQ(estimator.on_configure({}), CallbackReturn::SUCCESS);
    Hardware hardware;
    hardware.assign(estimator);
    auto exports = estimator.export_state_interfaces();
    ASSERT_EQ(estimator.on_activate({}), CallbackReturn::SUCCESS);
    if (failure == 0)
    {
      hardware.set("robot/imu/orientation.w", 0.0);
    }
    if (failure == 1)
    {
      hardware.set("left_sensor/state", .5);
    }
    if (failure == 2)
    {
      estimator.throw_input = true;
    }
    if (failure == 3)
    {
      estimator.resize_input = true;
    }
    EXPECT_EQ(step(estimator, 1.0, failure == 4 ? 0.0 : .01), Return::ERROR);
    EXPECT_DOUBLE_EQ(value(exports, estimator.get_name() + "/status/valid"), 0.0);
    EXPECT_FALSE(estimator.core().isinit);
  }
}

TEST_F(ControllerTest, MissingDuplicateInterfacesAndFailedHooksPreventActivation)
{
  for (int failure = 0; failure < 3; ++failure)
  {
    TestEstimator estimator;
    initialize(estimator, "activation" + std::to_string(failure));
    ASSERT_EQ(estimator.on_configure({}), CallbackReturn::SUCCESS);
    Hardware hardware;
    hardware.assign(estimator, failure == 0, failure == 1);
    if (failure == 2)
    {
      estimator.activate_result = CallbackReturn::ERROR;
    }
    EXPECT_EQ(estimator.on_activate({}), CallbackReturn::ERROR);
    EXPECT_EQ(step(estimator), Return::ERROR);
  }
  TestEstimator estimator;
  initialize(estimator, "configure_failure");
  estimator.configure_result = CallbackReturn::FAILURE;
  EXPECT_EQ(estimator.on_configure({}), CallbackReturn::FAILURE);
  EXPECT_FALSE(estimator.has_estimator());
  EXPECT_TRUE(estimator.raw_exports().empty());
}

TEST_F(ControllerTest, ModelDescriptionReconfigurationAndParameterValidation)
{
  TestEstimator estimator;
  auto params = configuration();
  params[0] = rclcpp::Parameter("model.path", "");
  initialize(estimator, "description", params, true);
  ASSERT_EQ(estimator.on_configure({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(estimator.on_cleanup({}), CallbackReturn::SUCCESS);
  ASSERT_TRUE(estimator.get_node()
                  ->set_parameter(rclcpp::Parameter("contact_names", std::vector<std::string>{"left_foot"}))
                  .successful);
  ASSERT_TRUE(
      estimator.get_node()
          ->set_parameter(rclcpp::Parameter("interfaces.contacts", std::vector<std::string>{"left_sensor/state"}))
          .successful);
  ASSERT_EQ(estimator.on_configure({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(estimator.raw_exports().size(), 28u);
  for (const auto& invalid : std::vector<rclcpp::Parameter>{
           rclcpp::Parameter("joint_names", std::vector<std::string>{"left_joint", "left_joint", "right_joint"}),
           rclcpp::Parameter("joint_position_std", std::vector<double>{-1.0}),
           rclcpp::Parameter("interfaces.joint_names", std::vector<std::string>{"one"}),
           rclcpp::Parameter("q_body_imu", std::vector<double>{0.0, 0.0, 0.0, 0.0}), rclcpp::Parameter("N", 0),
           rclcpp::Parameter("model.path", "relative.urdf"),
           rclcpp::Parameter("interfaces.contacts", std::vector<std::string>{"no_slash"}),
           rclcpp::Parameter("interfaces.imu", ""),
           rclcpp::Parameter("interfaces.contacts", std::vector<std::string>{"/state", "sensor/state"}),
           rclcpp::Parameter("interfaces.contacts", std::vector<std::string>{"sensor/", "sensor/state"})})
  {
    TestEstimator bad;
    auto overrides = configuration();
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
                                   [&](const auto& p) { return p.get_name() == invalid.get_name(); }),
                    overrides.end());
    overrides.push_back(invalid);
    initialize(bad, "bad", overrides);
    EXPECT_EQ(bad.on_configure({}), CallbackReturn::ERROR) << invalid.get_name();
    EXPECT_FALSE(bad.has_estimator());
  }
}

TEST_F(ControllerTest, OptionalMomentumObserverAndIndependentInstances)
{
  TestEstimator first, second;
  initialize(first, "first", configuration(true));
  initialize(second, "second", configuration(true));
  ASSERT_EQ(first.on_configure({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_configure({}), CallbackReturn::SUCCESS);
  EXPECT_EQ(first.state_interface_configuration().names.size(), 19u);
  auto first_exports = first.export_state_interfaces(), second_exports = second.export_state_interfaces();
  ASSERT_EQ(first_exports.size(), 32u);
  Hardware first_hardware, second_hardware;
  first_hardware.assign(first);
  second_hardware.assign(second);
  ASSERT_EQ(first.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_activate({}), CallbackReturn::SUCCESS);
  ASSERT_TRUE(first.set_chained_mode(true));
  ASSERT_EQ(step(first), Return::OK);
  ASSERT_EQ(step(second), Return::OK);
  const auto second_state = second.core().x_est_;
  first_hardware.set("robot/left/position", .8);
  first_hardware.set("robot/left/effort", 15.0);
  ASSERT_EQ(step(first, 2.0), Return::OK);
  EXPECT_TRUE(second.core().x_est_.isApprox(second_state, 0));
  EXPECT_DOUBLE_EQ(value(second_exports, "second/joints/left_joint/position"), .3);
  EXPECT_DOUBLE_EQ(value(first_exports, "first/joints/left_joint/position"), .8);
  EXPECT_DOUBLE_EQ(value(first_exports, "first/joints/left_joint/effort"), 15.0);
  ASSERT_EQ(first.on_deactivate({}), CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(value(second_exports, "second/status/valid"), 1.0);
}
}  // namespace
