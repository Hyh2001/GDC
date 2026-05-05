#include "base_humanoid_controllers/base_humanoid_controller.hpp"

namespace humanoid_controllers
{

controller_interface::CallbackReturn BaseHumanoidController::on_init()
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
  foot_names_ = auto_declare<std::vector<std::string>>("foot_names", foot_names_);
  foot_sensor_names_ = auto_declare<std::vector<std::string>>("foot_sensor_names", foot_sensor_names_);
  pos_name_ = auto_declare<std::string>("pos_name", pos_name_);
  ori_name_ = auto_declare<std::string>("ori_name", ori_name_);
  lin_vel_name_ = auto_declare<std::string>("lin_vel_name", lin_vel_name_);
  ang_vel_name_ = auto_declare<std::string>("ang_vel_name", ang_vel_name_);
  lin_acc_name_ = auto_declare<std::string>("lin_acc_name", lin_acc_name_);
  ang_acc_name_ = auto_declare<std::string>("ang_acc_name", ang_acc_name_);

  // disable interface lists must be declared explicitly to ensure YAML overrides are visible here.
  for (const auto & joint : joint_names_)
  {
    const auto disabled_cmd =
        auto_declare<std::vector<std::string>>("disable_command_interfaces." + joint, std::vector<std::string>{});
    if (!disabled_cmd.empty())
    {
      disabled_cmd_ifaces_[joint] = disabled_cmd;
    }

    const auto disabled_state =
        auto_declare<std::vector<std::string>>("disable_state_interfaces." + joint, std::vector<std::string>{});
    if (!disabled_state.empty())
    {
      disabled_state_ifaces_[joint] = disabled_state;
    }
  }

  // debug
  debug_ = auto_declare<bool>("debug", false);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn BaseHumanoidController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseHumanoidController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseHumanoidController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BaseHumanoidController::update_and_write_commands(const rclcpp::Time& time,
                                                                                    const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> BaseHumanoidController::on_export_reference_interfaces()
{
  return {};
}

controller_interface::return_type BaseHumanoidController::update_reference_from_subscribers(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_joint_state_interface_configuration() const
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

controller_interface::InterfaceConfiguration BaseHumanoidController::get_global_pos_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + pos_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + pos_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + pos_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_ori_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_w_est");
  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_global_lin_vel_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_global_ang_vel_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_global_lin_acc_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_global_ang_acc_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_z_est");
  return config;
}

/*
  @return: 2 * (contact_state)
*/
controller_interface::InterfaceConfiguration BaseHumanoidController::get_contact_state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& foot : foot_names_)
  {
    config.names.push_back(estimator_name_ + "/" + foot + "_" + foot_sensor_names_[0] + "_est");
  }

  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_contact_force_torque_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& foot : foot_names_)
  {
    for (size_t i = 1; i < foot_sensor_names_.size(); ++i) // skip contact state
    {
      const auto& sensor = foot_sensor_names_[i];
      config.names.push_back(estimator_name_ + "/" + foot + "_" + sensor + "_est");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidController::get_joint_command_interface_configuration() const
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

void BaseHumanoidController::read_global_pos_from_state_interfaces(std::array<double, 3>& pos) const
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + pos_name_ + "_x_est", pos[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + pos_name_ + "_y_est", pos[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + pos_name_ + "_z_est", pos[2]);
}

void BaseHumanoidController::read_ori_from_state_interfaces(std::array<double, 4>& ori) const
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_w_est", ori[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_x_est", ori[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_y_est", ori[2]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_z_est", ori[3]);
}

void BaseHumanoidController::read_global_lin_vel_from_state_interfaces(std::array<double, 3>& lin_vel) const
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_vel_name_ + "_x_est", lin_vel[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_vel_name_ + "_y_est", lin_vel[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_vel_name_ + "_z_est", lin_vel[2]);
}

void BaseHumanoidController::read_global_ang_vel_from_state_interfaces(std::array<double, 3>& ang_vel) const
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_vel_name_ + "_x_est", ang_vel[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_vel_name_ + "_y_est", ang_vel[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_vel_name_ + "_z_est", ang_vel[2]);
}

void BaseHumanoidController::read_global_lin_acc_from_state_interfaces(std::array<double, 3>& lin_acc) const
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_acc_name_ + "_x_est", lin_acc[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_acc_name_ + "_y_est", lin_acc[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_acc_name_ + "_z_est", lin_acc[2]);
}

void BaseHumanoidController::read_global_ang_acc_from_state_interfaces(std::array<double, 3>& ang_acc) const
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_acc_name_ + "_x_est", ang_acc[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_acc_name_ + "_y_est", ang_acc[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_acc_name_ + "_z_est", ang_acc[2]);
}

void BaseHumanoidController::read_contact_state_from_state_interfaces(std::array<bool, 2>& contact_state) const
{
  for (size_t i = 0; i < foot_names_.size(); ++i)
  {
    const auto& foot = foot_names_[i];
    const auto& sensor = foot_sensor_names_[0]; // contact state
    double value;
    base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + foot + "_" + sensor + "_est", value);
    contact_state[i] = value;
  }
}

void BaseHumanoidController::read_contact_force_torque_from_state_interfaces(std::array<std::array<double, 6>, 2>& contact_force_torque) const
{
  for (size_t i = 0; i < foot_names_.size(); ++i)
  {
    const auto& foot = foot_names_[i];
    for (size_t j = 1; j < foot_sensor_names_.size(); ++j) // skip contact state (j=0)
    {
      const auto& sensor = foot_sensor_names_[j];
      double value;
      base_utils::get_state_interface_value(state_interfaces_,
          estimator_name_ + "/" + foot + "_" + sensor + "_est", value);
      contact_force_torque[i][j-1] = value;  // j=1->index 0, j=2->index 1, etc.
    }
  }
}

void BaseHumanoidController::read_joint_states_from_state_interfaces(std::vector<double>& pos) const
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  pos.assign(joint_names_.size(), nan);

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
  }
}

void BaseHumanoidController::read_joint_states_from_state_interfaces(std::vector<double>& pos, std::vector<double>& vel) const
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  pos.assign(joint_names_.size(), nan);
  vel.assign(joint_names_.size(), nan);

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
  }
}

void BaseHumanoidController::read_joint_states_from_state_interfaces(std::vector<double>& pos, std::vector<double>& vel,
                                                                    std::vector<double>& tau) const
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

void BaseHumanoidController::write_joint_commands_to_command_interfaces(const std::vector<double>& pos, const std::vector<double>& vel,
                                                  const std::vector<double>& tau, const std::vector<double>& kp,
                                                  const std::vector<double>& kd)
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

};  // namespace humanoid_controllers
