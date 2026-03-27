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

  joint_pos_.resize(num_joints_, 0.0);
  joint_vel_.resize(num_joints_, 0.0);
  joint_acc_.resize(num_joints_, 0.0);
  joint_tau_.resize(num_joints_, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
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

std::vector<hardware_interface::StateInterface> BaseManipulatorEstimator::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::string estimator_name = this->get_name();
  size_t joint_idx = 0;
  // Joint interfaces
  for (const auto& joint : joint_names_)
  {
    for (const auto& iface : joint_state_interface_types_)
    {
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
    for (const auto& interface_type : joint_state_interface_types_)
    {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

void BaseManipulatorEstimator::read_joint_states_from_state_interfaces(std::vector<double>& pos, std::vector<double>& vel,
                                                                    std::vector<double>& tau) const
{
  pos.resize(num_joints_, 0.0);
  vel.resize(num_joints_, 0.0);
  tau.resize(num_joints_, 0.0);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint = joint_names_[i];
    base_utils::get_state_interface_value(state_interfaces_, joint + "/position", pos[i]);
    base_utils::get_state_interface_value(state_interfaces_, joint + "/velocity", vel[i]);
    base_utils::get_state_interface_value(state_interfaces_, joint + "/effort", tau[i]);
  }
}

}
