#include "base_humanoid_controllers/humanoid_passthrough_controller.hpp"

namespace humanoid_controllers
{

controller_interface::CallbackReturn HumanoidPassthroughController::on_init()
{
  controller_interface::CallbackReturn state = BaseHumanoidController::on_init();
  // check whether reference interfaces conflicted
  if (planner_name_ != "" && ref_controller_name_ != "")
  {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "HumanoidPDController::on_init() failed because both planner_name and ref_controller_name are set.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // resize vectors
  joint_pos_ref_.resize(joint_names_.size(), 0.0);
  joint_vel_ref_.resize(joint_names_.size(), 0.0);
  kp_gains_.resize(joint_names_.size(), 0.0);
  kd_gains_.resize(joint_names_.size(), 0.0);

  // init pd gains and targets from parameters if exist
  joint_pos_ref_ = auto_declare<std::vector<double>>("joint_pos_ref", joint_pos_ref_);
  joint_vel_ref_ = auto_declare<std::vector<double>>("joint_vel_ref", joint_vel_ref_);
  kp_gains_ = auto_declare<std::vector<double>>("kp_gains", kp_gains_);
  kd_gains_ = auto_declare<std::vector<double>>("kd_gains", kd_gains_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HumanoidPassthroughController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config =
    humanoid_controllers::BaseHumanoidController::get_joint_command_interface_configuration();

  return config;
}

controller_interface::InterfaceConfiguration HumanoidPassthroughController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // state estimation
  if (estimator_name_ != "")
  {
    // Joint interfaces
    for (const auto& joint : joint_names_)
    {
      for (const auto& iface : joint_state_interface_types_)
      {
        config.names.push_back(estimator_name_ + "/" + joint + "_" + iface + "_est");
      }
    }

    // Contact and force wrench sensors
    for (const auto& foot : foot_names_)
    {
      for (const auto& sensor : foot_sensor_names_)
      {
        config.names.push_back(estimator_name_ + "/" + foot + "_" + sensor + "_est");
      }
    }

    // Position (x, y, z)
    config.names.push_back(estimator_name_ + "/" + pos_name_ + "_x_est");
    config.names.push_back(estimator_name_ + "/" + pos_name_ + "_y_est");
    config.names.push_back(estimator_name_ + "/" + pos_name_ + "_z_est");

    // Orientation (w, x, y, z)
    config.names.push_back(estimator_name_ + "/" + ori_name_ + "_w_est");
    config.names.push_back(estimator_name_ + "/" + ori_name_ + "_x_est");
    config.names.push_back(estimator_name_ + "/" + ori_name_ + "_y_est");
    config.names.push_back(estimator_name_ + "/" + ori_name_ + "_z_est");

    // Linear velocity (x, y, z)
    config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_x_est");
    config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_y_est");
    config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_z_est");

    // Angular velocity (x, y, z)
    config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_x_est");
    config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_y_est");
    config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_z_est");

    // Linear acceleration (x, y, z)
    config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_x_est");
    config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_y_est");
    config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_z_est");

    // Angular acceleration (x, y, z)
    config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_x_est");
    config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_y_est");
    config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_z_est");
  }

  // reference trajectory
  if (planner_name_ != "")
  {
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(planner_name_ + "/" + joint_name + "_position_ref");
      config.names.push_back(planner_name_ + "/" + joint_name + "_velocity_ref");
    }
  }

  return config;
}

controller_interface::CallbackReturn HumanoidPassthroughController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  controller_interface::CallbackReturn state = BaseHumanoidController::on_configure(previous_state);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidPassthroughController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidPassthroughController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HumanoidPassthroughController::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)time;
  (void)period;

  const std::vector<double> joint_tau_ref(joint_names_.size(), 0.0);
  write_joint_commands_to_command_interfaces(
      joint_pos_ref_,
      joint_vel_ref_,
      joint_tau_ref,
      kp_gains_,
      kd_gains_);

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> HumanoidPassthroughController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  std::string controller_name = this->get_node()->get_name();
  reference_interfaces_.resize(joint_names_.size() * 2, 0.0);  // position and velocity refs
  // Export position reference interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    reference_interfaces.emplace_back(controller_name, joint_names_[i] + "_position_ref", &joint_pos_ref_[i]);
  }
  // Export velocity reference interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    reference_interfaces.emplace_back(controller_name, joint_names_[i] + "_velocity_ref", &joint_vel_ref_[i]);
  }

  return reference_interfaces;
}
};  // namespace humanoid_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(humanoid_controllers::HumanoidPassthroughController,
                       controller_interface::ChainableControllerInterface);
