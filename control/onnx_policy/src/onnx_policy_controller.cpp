#include "onnx_policy/onnx_policy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace onnx_policy
{
namespace
{
void require_finite(const std::vector<double>& values)
{
  for (double value : values)
    if (!std::isfinite(value))
      throw std::invalid_argument("Nonfinite policy buffer");
}
}  // namespace

OnnxPolicyController::CallbackReturn OnnxPolicyController::on_init()
{
  try
  {
    auto_declare<std::string>("model.path", "");
    auto_declare<std::string>("model.package", "");
    auto_declare<std::string>("model.type", "mlp");
    auto_declare<int>("model.hidden_layers", 0);
    auto_declare<int>("model.hidden_dim", 0);
    return on_policy_init();
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Policy initialization failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

OnnxPolicyController::CallbackReturn OnnxPolicyController::on_configure(const rclcpp_lifecycle::State& state)
{
  if (active_)
    return CallbackReturn::ERROR;
  release_policy();
  try
  {
    const auto result = on_policy_configure(state);
    if (result != CallbackReturn::SUCCESS)
    {
      release_policy();
      return result;
    }
    OnnxPolicyCfg cfg;
    cfg.input_size = observation_manager_.total_observation_dim();
    cfg.output_size = action_manager_.total_action_dim();
    if (cfg.input_size <= 0 || cfg.output_size <= 0)
      throw std::invalid_argument("Policy requires nonempty observation and action managers");
    std::filesystem::path path(get_node()->get_parameter("model.path").as_string());
    const auto package = get_node()->get_parameter("model.package").as_string();
    if (path.empty())
      throw std::invalid_argument("model.path is required");
    if (!path.is_absolute())
    {
      if (package.empty())
        throw std::invalid_argument("Relative model.path requires model.package");
      path = std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) / path;
    }
    if (!std::filesystem::is_regular_file(path))
      throw std::invalid_argument("Model file does not exist");
    cfg.model_path = path.string();
    cfg.validate_model_io = true;
    const auto type = get_node()->get_parameter("model.type").as_string();
    if (type == "mlp")
      policy_ = std::make_unique<OnnxPolicy>(cfg);
    else if (type == "rnn")
    {
      OnnxRNNPolicyCfg rnn;
      static_cast<OnnxPolicyCfg&>(rnn) = cfg;
      const auto layers = get_node()->get_parameter("model.hidden_layers").as_int();
      const auto dim = get_node()->get_parameter("model.hidden_dim").as_int();
      if (layers <= 0 || layers > std::numeric_limits<int>::max() || dim <= 0 || dim > std::numeric_limits<int>::max())
        throw std::invalid_argument("Invalid recurrent dimensions");
      rnn.hidden_layers = static_cast<int>(layers);
      rnn.hidden_dim = static_cast<int>(dim);
      policy_ = std::make_unique<OnnxRNN>(rnn);
    }
    else
      throw std::invalid_argument("model.type must be mlp or rnn");
    observations_.assign(cfg.input_size, 0.0);
    actions_.assign(cfg.output_size, 0.0);
    processed_actions_.assign(cfg.output_size, 0.0);
    pending_actions_.assign(cfg.output_size, 0.0);
    policy_->infer_into(observations_, pending_actions_);
    reset_policy_state();
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Policy configuration failed: %s", error.what());
    release_policy();
    return CallbackReturn::ERROR;
  }
}

OnnxPolicyController::CallbackReturn OnnxPolicyController::on_activate(const rclcpp_lifecycle::State& state)
{
  faulted_ = true;
  active_ = false;
  if (!policy_)
    return CallbackReturn::ERROR;
  try
  {
    reset_policy_state();
    const auto result = on_policy_activate(state);
    if (result != CallbackReturn::SUCCESS)
      return result;
    if (actions_.size() != pending_actions_.size())
      throw std::invalid_argument("Activation changed action dimension");
    require_finite(actions_);
    active_ = true;
    faulted_ = false;
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Policy activation failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

OnnxPolicyController::CallbackReturn OnnxPolicyController::on_deactivate(const rclcpp_lifecycle::State& state)
{
  active_ = false;
  faulted_ = true;
  CallbackReturn result = CallbackReturn::ERROR;
  try
  {
    result = on_policy_deactivate(state);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Policy deactivation failed: %s", error.what());
  }
  try
  {
    reset_policy_state();
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Policy reset failed: %s", error.what());
    result = CallbackReturn::ERROR;
  }
  return result;
}

OnnxPolicyController::CallbackReturn OnnxPolicyController::on_cleanup(const rclcpp_lifecycle::State& state)
{
  active_ = false;
  faulted_ = true;
  CallbackReturn result = CallbackReturn::ERROR;
  try
  {
    result = on_policy_cleanup(state);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Policy cleanup failed: %s", error.what());
  }
  release_policy();
  return result;
}

void OnnxPolicyController::reset_policy_state()
{
  observation_manager_.reset();
  action_manager_.reset();
  for (auto* values : {&observations_, &actions_, &processed_actions_, &pending_actions_})
    std::fill(values->begin(), values->end(), 0.0);
  if (policy_)
    policy_->reset();
}

void OnnxPolicyController::release_policy()
{
  active_ = false;
  faulted_ = true;
  policy_.reset();
  observation_manager_ = observation::ObservationManager();
  action_manager_ = action::ActionManager();
  observations_.clear();
  actions_.clear();
  processed_actions_.clear();
  pending_actions_.clear();
}

OnnxPolicyController::return_type OnnxPolicyController::fail_update(const char* reason)
{
  faulted_ = true;
  RCLCPP_ERROR(get_node()->get_logger(), "Policy update failed: %s", reason);
  return return_type::ERROR;
}

OnnxPolicyController::return_type OnnxPolicyController::update_and_write_commands(const rclcpp::Time& time,
                                                                                  const rclcpp::Duration& period)
{
  if (!active_ || faulted_ || !policy_)
    return return_type::ERROR;
  try
  {
    if (read_policy_inputs(time, period) != return_type::OK)
      return fail_update("Input hook returned ERROR");
    const auto& observations = observation_manager_.compute_view();
    if (observations.size() != observations_.size())
      throw std::invalid_argument("Observation dimension changed");
    require_finite(observations);
    std::copy(observations.begin(), observations.end(), observations_.begin());
    policy_->infer_into(observations_, pending_actions_);
    require_finite(pending_actions_);
    action_manager_.process_action(pending_actions_);
    const auto& processed = action_manager_.processed_actions_view();
    if (processed.size() != processed_actions_.size() || actions_.size() != pending_actions_.size())
      throw std::invalid_argument("Action dimension changed");
    require_finite(processed);
    std::copy(processed.begin(), processed.end(), processed_actions_.begin());
    if (write_policy_commands(time, period, processed_actions_) != return_type::OK)
      return fail_update("Command hook returned ERROR");
    std::copy(pending_actions_.begin(), pending_actions_.end(), actions_.begin());
    return return_type::OK;
  }
  catch (const std::exception& error)
  {
    return fail_update(error.what());
  }
}
}  // namespace onnx_policy
