#include "iekf_estimator/iekf_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace iekf_estimator
{
namespace
{
struct VectorParameter
{
  const char* name;
  std::vector<double> robot_params::*member;
  std::vector<double> initial;
};
const std::vector<VectorParameter>& vector_parameters()
{
  static const std::vector<VectorParameter> parameters = {
      {"p_init_std", &robot_params::p_init_std_, {.1, .1, .1}},
      {"v_init_std", &robot_params::v_init_std_, {.1, .1, .1}},
      {"accel_input_std", &robot_params::accel_input_std_, {.1, .1, .1}},
      {"gyro_input_std", &robot_params::gyro_input_std_, {.01, .01, .01}},
      {"accel_bias_process_std", &robot_params::accel_bias_process_std_, {.001, .001, .001}},
      {"angular_bias_process_std", &robot_params::angular_bias_process_std_, {.001, .001, .001}},
      {"joint_position_std", &robot_params::joint_position_std_, {.01}},
      {"joint_velocity_std", &robot_params::joint_velocity_std_, {.1}},
      {"foot_slide_std", &robot_params::foot_slide_std_, {.01, .01, .01}},
      {"foot_swing_std", &robot_params::foot_swing_std_, {1.0, 1.0, 1.0}},
      {"q_body_imu", &robot_params::q_body_imu_, {1.0, 0.0, 0.0, 0.0}},
      {"p_body_imu", &robot_params::p_body_imu_, {0.0, 0.0, 0.0}}};
  return parameters;
}
int checked_int(int64_t value, const char* name)
{
  if (value < 0 || value >= std::numeric_limits<int>::max())
  {
    throw std::invalid_argument(std::string(name) + " is outside the supported integer range");
  }
  return static_cast<int>(value);
}
}  // namespace

IEKFEstimator::CallbackReturn IEKFEstimator::on_init()
{
  try
  {
    auto_declare<std::string>("model.path", "");
    auto_declare<std::string>("model.package", "");
    auto_declare<std::vector<std::string>>("joint_names", {});
    auto_declare<std::vector<std::string>>("contact_names", {});
    auto_declare<std::vector<std::string>>("interfaces.joint_names", {});
    auto_declare<std::string>("interfaces.imu", "imu");
    auto_declare<std::vector<std::string>>("interfaces.contacts", {});
    auto_declare<int>("N", 1);
    auto_declare<int>("num_legs", 0);
    auto_declare<int>("dim_legs", 0);
    auto_declare<bool>("using_lo_p", true);
    auto_declare<bool>("using_lo_v", true);
    auto_declare<bool>("using_mob", false);
    auto_declare<bool>("visualize", false);
    auto_declare<double>("contact_effort_threshold", 10.0);
    auto_declare<double>("ground_height_offset", 0.05);
    for (const auto& parameter : vector_parameters())
    {
      auto_declare<std::vector<double>>(parameter.name, parameter.initial);
    }
    for (const auto* name : {"orientation_init_std", "foot_init_std", "accel_bias_init_std", "angular_bias_init_std"})
    {
      auto_declare<double>(name, .01);
    }
    return on_estimator_init();
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "IEKF initialization failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

void IEKFEstimator::load_parameters()
{
  auto node = get_node();
  parameters_ = std::make_shared<robot_params>();
  auto& p = *parameters_;
  p.joint_names_ = node->get_parameter("joint_names").as_string_array();
  p.contact_names_ = node->get_parameter("contact_names").as_string_array();
  p.N_ = checked_int(node->get_parameter("N").as_int(), "N");
  p.num_legs_ = checked_int(node->get_parameter("num_legs").as_int(), "num_legs");
  p.dim_legs_ = checked_int(node->get_parameter("dim_legs").as_int(), "dim_legs");
  p.rate_ = checked_int(get_update_rate(), "update_rate");
  p.using_lo_p_ = node->get_parameter("using_lo_p").as_bool();
  p.using_lo_v_ = node->get_parameter("using_lo_v").as_bool();
  p.using_mob_ = node->get_parameter("using_mob").as_bool();
  p.visualize_ = node->get_parameter("visualize").as_bool();
  p.contact_theshold_ = node->get_parameter("contact_effort_threshold").as_double();
  p.ground_height_offset_ = node->get_parameter("ground_height_offset").as_double();
  for (const auto& parameter : vector_parameters())
  {
    p.*(parameter.member) = node->get_parameter(parameter.name).as_double_array();
  }
  p.orientation_init_std_ = node->get_parameter("orientation_init_std").as_double();
  p.foot_init_std_ = node->get_parameter("foot_init_std").as_double();
  p.accel_bias_init_std_ = node->get_parameter("accel_bias_init_std").as_double();
  p.angular_bias_init_std_ = node->get_parameter("angular_bias_init_std").as_double();
  p.robot_name_ = get_name();

  model_ = pinocchio::Model();
  std::filesystem::path path(node->get_parameter("model.path").as_string());
  if (path.empty())
  {
    if (get_robot_description().empty())
    {
      throw std::invalid_argument("Provide model.path or the controller manager's robot_description");
    }
    pinocchio::urdf::buildModelFromXML(get_robot_description(), pinocchio::JointModelFreeFlyer(), model_);
  }
  else
  {
    if (!path.is_absolute())
    {
      const auto package = node->get_parameter("model.package").as_string();
      if (package.empty())
      {
        throw std::invalid_argument("Relative model.path requires model.package");
      }
      path = std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) / path;
    }
    if (!std::filesystem::is_regular_file(path))
    {
      throw std::invalid_argument("URDF model file does not exist");
    }
    p.urdf_path_ = path.string();
    pinocchio::urdf::buildModel(path.string(), pinocchio::JointModelFreeFlyer(), model_);
  }
  data_ = std::make_unique<pinocchio::Data>(model_);
  inputs_ = std::make_shared<robot_store>();
  inputs_->imu_time_ = 0.0;
  inputs_->accel_b_.setZero();
  inputs_->omega_b_.setZero();
  inputs_->quaternion_.setIdentity();
  inputs_->p_.setZero();
  inputs_->v_b_.setZero();
  inputs_->joint_states_position_ = Eigen::VectorXd::Zero(model_.nq - 7);
  inputs_->joint_states_velocity_ = Eigen::VectorXd::Zero(model_.nv - 6);
  inputs_->joint_states_effort_ = Eigen::VectorXd::Zero(model_.nv - 6);
  inputs_->contact_ = Eigen::VectorXd::Zero(p.contact_names_.size());
  estimator_ = std::make_unique<robot_IEKF>(model_, *data_, inputs_, parameters_);
  const auto& q = p.q_body_imu_;
  rotation_body_from_imu_ = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
}

void IEKFEstimator::configure_interfaces()
{
  auto resources = get_node()->get_parameter("interfaces.joint_names").as_string_array();
  if (resources.empty())
  {
    resources = parameters_->joint_names_;
  }
  if (resources.size() != parameters_->joint_names_.size())
  {
    throw std::invalid_argument("interfaces.joint_names must match joint_names in size and order");
  }
  std::unordered_set<std::string> names;
  auto request = [&](const std::string& name)
  {
    if (name.empty() || !names.insert(name).second)
    {
      throw std::invalid_argument("Empty or duplicate input interface: " + name);
    }
    requested_interfaces_.push_back(name);
    return requested_interfaces_.size() - 1;
  };
  for (size_t i = 0; i < resources.size(); ++i)
  {
    if (resources[i].empty())
    {
      throw std::invalid_argument("Joint interface resource cannot be empty");
    }
    const auto id = model_.getJointId(parameters_->joint_names_[i]);
    JointInput input{};
    input.position = request(resources[i] + "/position");
    input.velocity = request(resources[i] + "/velocity");
    if (parameters_->using_mob_)
    {
      input.effort = request(resources[i] + "/effort");
    }
    input.input_q = model_.joints[id].idx_q() - 7;
    input.input_v = model_.joints[id].idx_v() - 6;
    joint_inputs_.push_back(input);
  }
  const auto imu = get_node()->get_parameter("interfaces.imu").as_string();
  if (imu.empty())
  {
    throw std::invalid_argument("interfaces.imu cannot be empty");
  }
  const std::array<const char*, 10> imu_suffixes = {
      "orientation.w",         "orientation.x",        "orientation.y",      "orientation.z",
      "angular_velocity.x",    "angular_velocity.y",   "angular_velocity.z", "linear_acceleration.x",
      "linear_acceleration.y", "linear_acceleration.z"};
  for (size_t i = 0; i < imu_inputs_.size(); ++i)
  {
    imu_inputs_[i] = request(imu + "/" + imu_suffixes[i]);
  }
  const auto contacts = get_node()->get_parameter("interfaces.contacts").as_string_array();
  if (parameters_->using_mob_)
  {
    if (!contacts.empty())
    {
      throw std::invalid_argument("interfaces.contacts must be empty when using_mob is enabled");
    }
  }
  else
  {
    if (contacts.size() != parameters_->contact_names_.size())
    {
      throw std::invalid_argument("interfaces.contacts must provide one full interface name per contact");
    }
    for (const auto& name : contacts)
    {
      const auto slash = name.rfind('/');
      if (slash == std::string::npos || slash == 0 || slash + 1 == name.size())
      {
        throw std::invalid_argument("Contact inputs require full resource/interface names");
      }
      contact_inputs_.push_back(request(name));
    }
  }

  output_names_ = {"status/valid", "status/sample_time"};
  for (const auto* suffix : {"x", "y", "z"})
  {
    output_names_.push_back(std::string("base/position.") + suffix);
  }
  for (const auto* suffix : {"w", "x", "y", "z"})
  {
    output_names_.push_back(std::string("base/orientation.") + suffix);
  }
  for (const auto* quantity :
       {"base/linear_velocity", "base/angular_velocity", "imu/gyroscope_bias", "imu/accelerometer_bias"})
  {
    for (const auto* suffix : {"x", "y", "z"})
    {
      output_names_.push_back(std::string(quantity) + "." + suffix);
    }
  }
  for (const auto& name : parameters_->joint_names_)
  {
    output_names_.push_back("joints/" + name + "/position");
    output_names_.push_back("joints/" + name + "/velocity");
    if (parameters_->using_mob_)
    {
      output_names_.push_back("joints/" + name + "/effort");
    }
  }
  for (const auto& name : parameters_->contact_names_)
  {
    output_names_.push_back("contacts/" + name + "/state");
  }
  outputs_.resize(output_names_.size());
  pending_outputs_.resize(output_names_.size());
  invalidate_outputs();
}

IEKFEstimator::CallbackReturn IEKFEstimator::on_configure(const rclcpp_lifecycle::State& state)
{
  if (active_)
  {
    return CallbackReturn::ERROR;
  }
  release_estimator();
  try
  {
    load_parameters();
    configure_interfaces();
    const auto result = on_estimator_configure(state);
    if (result != CallbackReturn::SUCCESS)
    {
      release_estimator();
      return result;
    }
    configured_ = true;
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "IEKF configuration failed: %s", error.what());
    release_estimator();
    return CallbackReturn::ERROR;
  }
}

controller_interface::InterfaceConfiguration IEKFEstimator::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}
controller_interface::InterfaceConfiguration IEKFEstimator::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, requested_interfaces_};
}

void IEKFEstimator::bind_interfaces()
{
  if (state_interfaces_.size() != requested_interfaces_.size() || !command_interfaces_.empty())
  {
    throw std::invalid_argument("Assigned interfaces do not match the IEKF input configuration");
  }
  std::unordered_map<std::string, size_t> indices;
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    if (!indices.emplace(state_interfaces_[i].get_name(), i).second)
    {
      throw std::invalid_argument("Duplicate assigned state interface");
    }
  }
  bound_interfaces_.clear();
  for (const auto& name : requested_interfaces_)
  {
    const auto found = indices.find(name);
    if (found == indices.end())
    {
      throw std::invalid_argument("Missing state interface: " + name);
    }
    bound_interfaces_.push_back(found->second);
  }
}

IEKFEstimator::CallbackReturn IEKFEstimator::on_activate(const rclcpp_lifecycle::State& state)
{
  active_ = false;
  faulted_ = true;
  invalidate_outputs();
  if (!configured_ || !estimator_)
  {
    return CallbackReturn::ERROR;
  }
  try
  {
    bind_interfaces();
    estimator_->reset();
    const auto result = on_estimator_activate(state);
    if (result != CallbackReturn::SUCCESS)
    {
      return result;
    }
    active_ = true;
    faulted_ = false;
    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "IEKF activation failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

IEKFEstimator::CallbackReturn IEKFEstimator::on_deactivate(const rclcpp_lifecycle::State& state)
{
  active_ = false;
  faulted_ = true;
  invalidate_outputs();
  auto result = CallbackReturn::ERROR;
  try
  {
    result = on_estimator_deactivate(state);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "IEKF deactivation hook failed: %s", error.what());
  }
  bound_interfaces_.clear();
  try
  {
    if (estimator_)
    {
      estimator_->reset();
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "IEKF reset failed: %s", error.what());
    result = CallbackReturn::ERROR;
  }
  return result;
}

IEKFEstimator::CallbackReturn IEKFEstimator::on_cleanup(const rclcpp_lifecycle::State& state)
{
  active_ = false;
  faulted_ = true;
  auto result = CallbackReturn::ERROR;
  try
  {
    result = on_estimator_cleanup(state);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "IEKF cleanup hook failed: %s", error.what());
  }
  release_estimator();
  return result;
}

void IEKFEstimator::invalidate_outputs()
{
  std::fill(outputs_.begin(), outputs_.end(), std::numeric_limits<double>::quiet_NaN());
  if (!outputs_.empty())
  {
    outputs_[0] = 0.0;
  }
}
void IEKFEstimator::release_estimator()
{
  configured_ = active_ = false;
  faulted_ = true;
  invalidate_outputs();
  estimator_.reset();
  inputs_.reset();
  parameters_.reset();
  data_.reset();
  joint_inputs_.clear();
  contact_inputs_.clear();
  bound_interfaces_.clear();
  requested_interfaces_.clear();
  // Retain exported storage until the next configure; activation never resizes it.
}

std::vector<hardware_interface::StateInterface> IEKFEstimator::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  if (configured_)
  {
    for (size_t i = 0; i < output_names_.size(); ++i)
    {
      interfaces.emplace_back(get_name(), output_names_[i], &outputs_[i]);
    }
  }
  return interfaces;
}

void IEKFEstimator::read_interfaces(const rclcpp::Time& time)
{
  auto read = [&](size_t index)
  {
    const auto value = state_interfaces_[bound_interfaces_[index]].get_optional();
    if (!value || !std::isfinite(*value))
    {
      throw std::invalid_argument("Unavailable or nonfinite state interface: " + requested_interfaces_[index]);
    }
    return *value;
  };
  for (const auto& joint : joint_inputs_)
  {
    inputs_->joint_states_position_[joint.input_q] = read(joint.position);
    inputs_->joint_states_velocity_[joint.input_v] = read(joint.velocity);
    if (parameters_->using_mob_)
    {
      inputs_->joint_states_effort_[joint.input_v] = read(joint.effort);
    }
  }
  std::array<double, 10> imu{};
  for (size_t i = 0; i < imu.size(); ++i)
  {
    imu[i] = read(imu_inputs_[i]);
  }
  Eigen::Quaterniond orientation(imu[0], imu[1], imu[2], imu[3]);
  if (!std::isfinite(orientation.norm()) || orientation.norm() <= 1e-12)
  {
    throw std::invalid_argument("IMU orientation quaternion must be finite and nonzero");
  }
  inputs_->quaternion_ = orientation.normalized() * Eigen::Quaterniond(rotation_body_from_imu_.transpose());
  inputs_->omega_b_ = rotation_body_from_imu_ * Eigen::Vector3d(imu[4], imu[5], imu[6]);
  inputs_->accel_b_ = rotation_body_from_imu_ * Eigen::Vector3d(imu[7], imu[8], imu[9]);
  for (size_t i = 0; i < contact_inputs_.size(); ++i)
  {
    inputs_->contact_[i] = read(contact_inputs_[i]);
  }
  inputs_->imu_time_ = time.seconds();
}

void IEKFEstimator::prepare_output()
{
  const auto& x = estimator_->x_est_;
  if (x.size() != 16 || !x.allFinite() || !estimator_->v_b_est_.allFinite() ||
      estimator_->contact_.size() != static_cast<int>(parameters_->contact_names_.size()) ||
      !estimator_->contact_.allFinite())
  {
    throw std::runtime_error("IEKF produced an invalid estimate");
  }
  Eigen::Quaterniond q(x[6], x[7], x[8], x[9]);
  if (!std::isfinite(q.norm()) || q.norm() <= 1e-12)
  {
    throw std::runtime_error("IEKF produced an invalid orientation");
  }
  q.normalize();
  if (outputs_[0] == 1.0)
  {
    const Eigen::Quaterniond previous(outputs_[5], outputs_[6], outputs_[7], outputs_[8]);
    if (q.coeffs().dot(previous.coeffs()) < 0.0)
    {
      q.coeffs() *= -1.0;
    }
  }
  const Eigen::Vector3d offset(parameters_->p_body_imu_[0], parameters_->p_body_imu_[1], parameters_->p_body_imu_[2]);
  const Eigen::Vector3d position = x.head<3>() - q.toRotationMatrix() * offset;
  const Eigen::Vector3d omega = inputs_->omega_b_ - x.segment<3>(10);
  size_t index = 0;
  pending_outputs_[index++] = 1.0;
  pending_outputs_[index++] = inputs_->imu_time_;
  auto append = [&](const auto& values)
  {
    for (Eigen::Index i = 0; i < values.size(); ++i)
    {
      pending_outputs_[index++] = values[i];
    }
  };
  append(position);
  pending_outputs_[index++] = q.w();
  append(q.vec());
  append(estimator_->v_b_est_);
  append(omega);
  append(x.segment<3>(10));
  append(x.segment<3>(13));
  for (const auto& joint : joint_inputs_)
  {
    pending_outputs_[index++] = inputs_->joint_states_position_[joint.input_q];
    pending_outputs_[index++] = inputs_->joint_states_velocity_[joint.input_v];
    if (parameters_->using_mob_)
    {
      pending_outputs_[index++] = inputs_->joint_states_effort_[joint.input_v];
    }
  }
  append(estimator_->contact_);
  if (index != pending_outputs_.size() ||
      !std::all_of(pending_outputs_.begin(), pending_outputs_.end(), [](double value) { return std::isfinite(value); }))
  {
    throw std::runtime_error("IEKF output buffer is invalid");
  }
}

IEKFEstimator::return_type IEKFEstimator::fail_update(const char* reason)
{
  faulted_ = true;
  if (!outputs_.empty())
  {
    outputs_[0] = 0.0;
  }
  RCLCPP_ERROR(get_node()->get_logger(), "IEKF update failed: %s", reason);
  return return_type::ERROR;
}
IEKFEstimator::return_type IEKFEstimator::update_and_write_commands(const rclcpp::Time& time,
                                                                    const rclcpp::Duration& period)
{
  if (!active_ || faulted_ || !estimator_)
  {
    return return_type::ERROR;
  }
  try
  {
    if (!std::isfinite(period.seconds()) || period.seconds() <= 0.0)
    {
      throw std::invalid_argument("IEKF update period must be positive");
    }
    read_interfaces(time);
    if (prepare_estimator_inputs(time, period) != return_type::OK)
    {
      return fail_update("Input preparation hook returned ERROR");
    }
    if (!std::isfinite(inputs_->imu_time_))
    {
      throw std::invalid_argument("Invalid measurement timestamp");
    }
    if (!parameters_->using_mob_)
    {
      for (Eigen::Index i = 0; i < inputs_->contact_.size(); ++i)
      {
        if (inputs_->contact_[i] != 0.0 && inputs_->contact_[i] != 1.0)
        {
          throw std::invalid_argument("Contact flags must be zero or one");
        }
      }
    }
    if (!estimator_->isinit)
    {
      estimator_->initialize();
    }
    else
    {
      estimator_->update(0);
    }
    prepare_output();
    std::copy(pending_outputs_.begin(), pending_outputs_.end(), outputs_.begin());
    return return_type::OK;
  }
  catch (const std::exception& error)
  {
    return fail_update(error.what());
  }
}
}  // namespace iekf_estimator
