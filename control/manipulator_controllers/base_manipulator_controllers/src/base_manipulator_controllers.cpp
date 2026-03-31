#include "base_manipulator_controllers/base_manipulator_controllers.hpp"

namespace manipulator_controllers
{

controller_interface::CallbackReturn BaseManipulatorController::on_init()
{
  // predecessors
  estimator_name_ = auto_declare<std::string>("estimator_name", estimator_name_);
  planner_name_ = auto_declare<std::string>("planner_name", planner_name_);
  ref_controller_name_ = auto_declare<std::string>("ref_controller_name", ref_controller_name_);

  // interfaces
  joint_names_ = auto_declare<std::vector<std::string>>("joint_names", joint_names_);
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

  // debug
  debug_ = auto_declare<bool>("debug", false);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BaseManipulatorController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration BaseManipulatorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn BaseManipulatorController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseManipulatorController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseManipulatorController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> BaseManipulatorController::on_export_reference_interfaces()
{
  return {};
}

controller_interface::return_type BaseManipulatorController::update_and_write_commands(const rclcpp::Time& time,
                                                                                          const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type BaseManipulatorController::update_reference_from_subscribers(const rclcpp::Time& time,
                                                                                          const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration BaseManipulatorController::get_joint_state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint : joint_names_)
  {
    const auto disabled_it = disabled_state_ifaces_.find(joint);
    for (const auto& iface : joint_state_interface_types_)
    {
      const bool disabled =
          (disabled_it != disabled_state_ifaces_.end()) &&
          (std::find(disabled_it->second.begin(), disabled_it->second.end(), iface) != disabled_it->second.end());
      if (!disabled)
      {
        config.names.push_back(estimator_name_ + "/" + joint + "_" + iface + "_est");
      }
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration BaseManipulatorController::get_joint_command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint : joint_names_)
  {
    const auto disabled_it = disabled_cmd_ifaces_.find(joint);
    for (const auto& iface_type : joint_command_interface_types_)
    {
      const bool disabled =
          (disabled_it != disabled_cmd_ifaces_.end()) &&
          (std::find(disabled_it->second.begin(), disabled_it->second.end(), iface_type) != disabled_it->second.end());
      if (!disabled)
      {
        config.names.push_back(joint + "/" + iface_type);
      }
    }
  }

  return config;
}

void BaseManipulatorController::read_joint_states_from_state_interfaces(
    std::vector<double>& pos, std::vector<double>& vel, std::vector<double>& tau) const
{
  // interfaces that are disabled will be filled with NaN
  const double nan = std::numeric_limits<double>::quiet_NaN();
  pos.assign(joint_names_.size(), nan);
  vel.assign(joint_names_.size(), nan);
  tau.assign(joint_names_.size(), nan);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint = joint_names_[i];
    const auto disabled_it = disabled_state_ifaces_.find(joint);
    const auto is_disabled = [&](const char* iface) {
      return (disabled_it != disabled_state_ifaces_.end()) &&
             (std::find(disabled_it->second.begin(), disabled_it->second.end(), iface) != disabled_it->second.end());
    };

    if (!is_disabled("position"))
      base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + joint + "_position_est", pos[i]);

    if (!is_disabled("velocity"))
      base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + joint + "_velocity_est", vel[i]);

    if (!is_disabled("effort"))
      base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + joint + "_effort_est", tau[i]);
  }
}

void BaseManipulatorController::write_joint_commands_to_command_interfaces(
    const std::vector<double>& pos, const std::vector<double>& vel, const std::vector<double>& tau,
    const std::vector<double>& kp, const std::vector<double>& kd)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint = joint_names_[i];
    const auto disabled_it = disabled_cmd_ifaces_.find(joint);
    const auto is_disabled = [&](const std::string& iface) {
      return (disabled_it != disabled_cmd_ifaces_.end()) &&
             (std::find(disabled_it->second.begin(), disabled_it->second.end(), iface) != disabled_it->second.end());
    };

    for (const auto& iface_type : joint_command_interface_types_)
    {
      if (is_disabled(iface_type))
        continue;

      double command_value = 0.0;
      if (iface_type == "position") command_value = pos[i];
      else if (iface_type == "velocity") command_value = vel[i];
      else if (iface_type == "effort") command_value = tau[i];
      else if (iface_type == "kp") command_value = kp[i];
      else if (iface_type == "kd") command_value = kd[i];
      else throw std::runtime_error("Unknown interface type: " + iface_type);

      base_utils::set_command_interface_value(command_interfaces_, joint + "/" + iface_type, command_value);
    }
  }
}


}; // namespace manipulator_controllers
