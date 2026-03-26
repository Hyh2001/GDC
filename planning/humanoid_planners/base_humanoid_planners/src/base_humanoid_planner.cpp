#include "base_humanoid_planners/base_humanoid_planner.hpp"

namespace humanoid_planners
{

controller_interface::CallbackReturn BaseHumanoidPlanner::on_init()
{
  // estimator name
  estimator_name_ = auto_declare<std::string>("estimator_name", estimator_name_);
  ref_planner_name_ = auto_declare<std::string>("ref_planner_name", ref_planner_name_);

  // interface names for parameters
  joint_names_ = auto_declare<std::vector<std::string>>("joint_names", joint_names_);
  num_joints_ = joint_names_.size();
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

  joint_pos_.resize(num_joints_, 0.0);
  joint_vel_.resize(num_joints_, 0.0);
  joint_acc_.resize(num_joints_, 0.0);
  joint_tau_.resize(num_joints_, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn BaseHumanoidPlanner::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseHumanoidPlanner::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseHumanoidPlanner::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BaseHumanoidPlanner::on_export_state_interfaces()
{
  return {};
}

controller_interface::return_type BaseHumanoidPlanner::update_and_write_commands(const rclcpp::Time& time,
                                                                                 const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_joint_state_interface_configuration() const
{
   controller_interface::InterfaceConfiguration config;
   config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

   for (const auto& joint : joint_names_)
   {
     for (const auto& iface : joint_state_interface_types_)
     {
       config.names.push_back(estimator_name_ + "/" + joint + "_" + iface + "_est");
     }
   }

   return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_global_pos_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + pos_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + pos_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + pos_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_ori_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_w_est");
  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + ori_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_global_lin_vel_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_global_ang_vel_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_global_lin_acc_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_x_est");
  config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_y_est");
  config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_z_est");
  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_global_ang_acc_interface_configuration() const
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
controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_contact_state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& foot : foot_names_)
  {
    for (const auto& sensor : foot_sensor_names_[0])
    {
      config.names.push_back(estimator_name_ + "/" + foot + "_" + sensor + "_est");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_contact_force_torque_interface_configuration() const
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

controller_interface::InterfaceConfiguration BaseHumanoidPlanner::get_joint_command_interface_configuration() const
{
   controller_interface::InterfaceConfiguration config;
   config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

   for (const auto& joint : joint_names_)
   {
     for (const auto& iface_type : joint_command_interface_types_)
     {
       config.names.push_back(joint + "/" + iface_type);
     }
   }

   return config;
}

void BaseHumanoidPlanner::read_global_pos_from_state_interfaces(std::array<double, 3>& pos)
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + pos_name_ + "_x_est", pos[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + pos_name_ + "_y_est", pos[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + pos_name_ + "_z_est", pos[2]);
}

void BaseHumanoidPlanner::read_ori_from_state_interfaces(std::array<double, 4>& ori)
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_w_est", ori[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_x_est", ori[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_y_est", ori[2]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ori_name_ + "_z_est", ori[3]);
}

void BaseHumanoidPlanner::read_global_lin_vel_from_state_interfaces(std::array<double, 3>& lin_vel)
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_vel_name_ + "_x_est", lin_vel[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_vel_name_ + "_y_est", lin_vel[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_vel_name_ + "_z_est", lin_vel[2]);
}

void BaseHumanoidPlanner::read_global_ang_vel_from_state_interfaces(std::array<double, 3>& ang_vel)
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_vel_name_ + "_x_est", ang_vel[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_vel_name_ + "_y_est", ang_vel[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_vel_name_ + "_z_est", ang_vel[2]);
}

void BaseHumanoidPlanner::read_global_lin_acc_from_state_interfaces(std::array<double, 3>& lin_acc)
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_acc_name_ + "_x_est", lin_acc[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_acc_name_ + "_y_est", lin_acc[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + lin_acc_name_ + "_z_est", lin_acc[2]);
}

void BaseHumanoidPlanner::read_global_ang_acc_from_state_interfaces(std::array<double, 3>& ang_acc)
{
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_acc_name_ + "_x_est", ang_acc[0]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_acc_name_ + "_y_est", ang_acc[1]);
  base_utils::get_state_interface_value(state_interfaces_, estimator_name_ + "/" + ang_acc_name_ + "_z_est", ang_acc[2]);
}

void BaseHumanoidPlanner::read_contact_state_from_state_interfaces(std::array<bool, 2>& contact_state)
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

void BaseHumanoidPlanner::read_contact_force_torque_from_state_interfaces(std::array<std::array<double, 6>, 2>& contact_force_torque)
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

void BaseHumanoidPlanner::read_joint_states_from_state_interfaces(
    std::vector<double>& pos, std::vector<double>& vel, std::vector<double>& tau) const
{
  pos.resize(num_joints_, 0.0);
  vel.resize(num_joints_, 0.0);
  tau.resize(num_joints_, 0.0);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint = joint_names_[i];
    base_utils::get_state_interface_value(
        state_interfaces_, estimator_name_ + "/" + joint + "_position_est", pos[i]);
    base_utils::get_state_interface_value(
        state_interfaces_, estimator_name_ + "/" + joint + "_velocity_est", vel[i]);
    base_utils::get_state_interface_value(
        state_interfaces_, estimator_name_ + "/" + joint + "_effort_est", tau[i]);
  }
}

};  // namespace humanoid_planners
