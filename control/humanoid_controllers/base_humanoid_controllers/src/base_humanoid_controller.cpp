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

// void BaseHumanoidController::get_state_interface_value(
//     const std::string & interface_name, double & value)
// {
//     for (const auto &state_iface : state_interfaces_) {
//         const std::string full_name = state_iface.get_prefix_name() + "/" + state_iface.get_interface_name();
//         if (full_name == interface_name) {
//             auto opt_value = state_iface.get_optional();
//             if (opt_value.has_value()) {
//                 value = opt_value.value();
//                 return;
//             }
//         }
//     }
//     throw std::runtime_error("State interface '" + interface_name + "' not found.");
// }

// void BaseHumanoidController::set_command_interface_value(
//     const std::string & interface_name, const double & value)
// {
//     for (auto &cmd_iface : command_interfaces_) {
//         const std::string full_name = cmd_iface.get_prefix_name() + "/" + cmd_iface.get_interface_name();
//         if (full_name == interface_name) {
//             cmd_iface.set_value(value);
//             return;
//         }
//     }
//     throw std::runtime_error("Command interface '" + interface_name + "' not found.");
// }

// void BaseHumanoidController::get_state_interface_values(
//     const std::vector<std::string> & interface_names,
//     std::vector<double> & values)
// {
//     if (values.size() != interface_names.size()) {
//         values.resize(interface_names.size());
//         RCLCPP_WARN(
//             this->get_node()->get_logger(),
//             "Resized values vector to match size of interface names.");
//     }
//     for (std::size_t i = 0; i < interface_names.size(); ++i) {
//         get_state_interface_value(interface_names[i], values[i]);
//     }
// }

// void BaseHumanoidController::set_command_interface_values(
//     const std::vector<std::string> & interface_names,
//     const std::vector<double> & values)
// {
//     if (values.size() != interface_names.size()) {
//         throw std::runtime_error("Size of values does not match size of interface names.");
//     }
//     for (std::size_t i = 0; i < interface_names.size(); ++i) {
//         set_command_interface_value(interface_names[i], values[i]);
//     }
// }
};  // namespace humanoid_controllers
