#include "base_manipulator_estimators/base_manipulator_estimator.hpp"

namespace manipulator_estimators
{
controller_interface::CallbackReturn BaseManipulatorEstimator::on_init()
{
  // interface names for parameters
  joint_names_ = auto_declare<std::vector<std::string>>("joint_names", joint_names_);
  num_joints_ = joint_names_.size();
  joint_state_interface_types_ =
      auto_declare<std::vector<std::string>>("joint_state_interfaces", joint_state_interface_types_);
  joint_command_interface_types_ =
      auto_declare<std::vector<std::string>>("joint_command_interfaces", joint_command_interface_types_);
  // disable certain interfaces that does not exist
  std::map<std::string, rclcpp::Parameter> cmd_exceptions;
  get_node()->get_node_parameters_interface()->get_parameters_by_prefix("disable_command_interfaces", cmd_exceptions);
  for (const auto & [joint, param] : cmd_exceptions) {
    disabled_cmd_ifaces_[joint] = param.as_string_array();
  }

  std::map<std::string, rclcpp::Parameter> state_exceptions;
  get_node()->get_node_parameters_interface()->get_parameters_by_prefix("disable_state_interfaces", state_exceptions);
  for (const auto & [joint, param] : state_exceptions) {
    disabled_state_ifaces_[joint] = param.as_string_array();
  }

  joint_pos_.resize(num_joints_, 0.0);
  joint_vel_.resize(num_joints_, 0.0);
  joint_acc_.resize(num_joints_, 0.0);
  joint_tau_.resize(num_joints_, 0.0);

  // debug
  debug_ = auto_declare<bool>("debug", false);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BaseManipulatorEstimator::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration BaseManipulatorEstimator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn BaseManipulatorEstimator::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseManipulatorEstimator::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseManipulatorEstimator::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BaseManipulatorEstimator::update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> BaseManipulatorEstimator::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::string estimator_name = this->get_name();
  size_t joint_idx = 0;
  // Joint interfaces
  for (const auto& joint : joint_names_)
  {
    const auto disabled_it = disabled_state_ifaces_.find(joint);
    for (const auto& iface : joint_state_interface_types_)
    {
      const bool disabled =
          (disabled_it != disabled_state_ifaces_.end()) &&
          (std::find(disabled_it->second.begin(), disabled_it->second.end(), iface) != disabled_it->second.end());
      if (disabled)
      {
        continue;
      }

      if (iface == "position")
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(estimator_name, joint + "_" + iface + "_est", &joint_pos_[joint_idx]));
      }
      else if (iface == "velocity")
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(estimator_name, joint + "_" + iface + "_est", &joint_vel_[joint_idx]));
      }
      else if (iface == "effort")
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(estimator_name, joint + "_" + iface + "_est", &joint_tau_[joint_idx]));
      }
    }
    joint_idx++;
  }

  return state_interfaces;
}

controller_interface::InterfaceConfiguration BaseManipulatorEstimator::get_joint_state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_)
  {
    const auto disabled_it = disabled_state_ifaces_.find(joint_name);
    for (const auto& interface_type : joint_state_interface_types_)
    {
      const bool disabled =
          (disabled_it != disabled_state_ifaces_.end()) &&
          (std::find(disabled_it->second.begin(), disabled_it->second.end(), interface_type) != disabled_it->second.end());
      if (!disabled)
      {
        config.names.push_back(joint_name + "/" + interface_type);
      }
    }
  }

  return config;
}

void BaseManipulatorEstimator::read_joint_states_from_state_interfaces(std::vector<double>& pos, std::vector<double>& vel,
                                                                    std::vector<double>& tau) const
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  pos.assign(num_joints_, nan);
  vel.assign(num_joints_, nan);
  tau.assign(num_joints_, nan);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint = joint_names_[i];
    const auto disabled_it = disabled_state_ifaces_.find(joint);
    const auto is_disabled = [&](const char* iface) {
      return (disabled_it != disabled_state_ifaces_.end()) &&
             (std::find(disabled_it->second.begin(), disabled_it->second.end(), iface) != disabled_it->second.end());
    };

    if (!is_disabled("position"))
      base_utils::get_state_interface_value(state_interfaces_, joint + "/position", pos[i]);

    if (!is_disabled("velocity"))
      base_utils::get_state_interface_value(state_interfaces_, joint + "/velocity", vel[i]);

    if (!is_disabled("effort"))
      base_utils::get_state_interface_value(state_interfaces_, joint + "/effort", tau[i]);
  }
}

}
