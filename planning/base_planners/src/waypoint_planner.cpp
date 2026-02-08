#include "base_planners/waypoint_planner.hpp"

namespace base_planners
{
controller_interface::CallbackReturn WaypointPlanner::on_init()
{
  std::vector<std::string> waypoint_names_ = auto_declare<std::vector<std::string>>("waypoints", {});
  waypoints_.resize(waypoint_names_.size());
  for (size_t i = 0; i < waypoint_names_.size(); ++i)
  {
    waypoints_[i].name = waypoint_names_[i];
    waypoints_[i].position = Eigen::Vector3d::Zero();
    waypoints_[i].orientation = Eigen::Quaterniond::Identity();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration WaypointPlanner::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration WaypointPlanner::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn WaypointPlanner::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WaypointPlanner::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WaypointPlanner::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WaypointPlanner::update_and_write_commands(const rclcpp::Time& time,
                                                                             const rclcpp::Duration& period)
{
  // TODOL fill in the logic
  return controller_interface::return_type::OK;
}

controller_interface::return_type WaypointPlanner::update_reference_from_subscribers(const rclcpp::Time& time,
                                                                                     const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> WaypointPlanner::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::string planner_name = this->get_name();
  for (auto& waypoint : waypoints_)
  {
    for (size_t i = 0; i < entry_names_.size(); ++i)
    {
      const auto& entry_name = entry_names_[i];
      std::string interface_name = waypoint.name + "/" + entry_name;
      double* data_ptr = nullptr;
      if (i < 3)
      {
        data_ptr = waypoint.position.data() + i;  // x,y,z
      }
      else
      {
        // Eigen::Quaterniond::coeffs() is [x, y, z, w]
        const size_t qi = i - 3;
        const size_t coeff_idx = (qi == 0) ? 3 : (qi - 1);  // w,x,y,z -> 3,0,1,2
        data_ptr = waypoint.orientation.coeffs().data() + coeff_idx;
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(planner_name, interface_name, data_ptr));
    }
  }
  return state_interfaces;
}

};  // namespace base_planners
