#pragma once
#ifndef BASE_PLANNERS_WAYPOINT_PLANNER_HPP__
#define BASE_PLANNERS_WAYPOINT_PLANNER_HPP__

#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "Eigen/Dense"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "base_utils/ros2_control_utils.hpp"

namespace base_planners
{
/*
    WaypointPlanner implements a waypoint planner taking a sequence of pose commands and pass them to
    the downstream controllers.
*/
struct Waypoint
{
  std::string name = "";
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();

  // static helper for full state representation (19 entries)
  static const std::vector<std::string>& get_entry_names()
  {
    static const std::vector<std::string> full_entry_names = {
      "pos_x", "pos_y", "pos_z",                              // position (0-2)
      "quat_w", "quat_x", "quat_y", "quat_z",                 // orientation (3-6)
      "lin_vel_x", "lin_vel_y", "lin_vel_z",                  // linear velocity (7-9)
      "ang_vel_x", "ang_vel_y", "ang_vel_z",                  // angular velocity (10-12)
      "lin_acc_x", "lin_acc_y", "lin_acc_z",                  // linear acceleration (13-15)
      "ang_acc_x", "ang_acc_y", "ang_acc_z"                   // angular acceleration (16-18)
    };
    return full_entry_names;
  }

  // helper for exporting state and command interfaces for waypoints
  static controller_interface::InterfaceConfiguration get_state_interface_configuration(
      const std::string& planner_name,
      const std::vector<std::string>& waypoint_names)
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& waypoint_name : waypoint_names)
    {
      for (const auto& entry_name : get_entry_names())
      {
        config.names.push_back(planner_name + "/" + waypoint_name + "/" + entry_name);
      }
    }
    return config;
  }

  // helper being used with on_export_state_interfaces and on_export_reference_interfaces
  static void append_state_interfaces(
      const std::string& planner_name,
      Waypoint& waypoint,
      std::vector<hardware_interface::StateInterface>& state_interfaces)
  {
    for (size_t i = 0; i < get_entry_names().size(); ++i)
    {
      const auto& entry_name = get_entry_names()[i];
      std::string interface_name = waypoint.name + "/" + entry_name;
      double* data_ptr = nullptr;
      if (i < 3)
      {
        data_ptr = waypoint.position.data() + i;
      }
      else if (i < 7)
      {
        const size_t qi = i - 3;
        const size_t coeff_idx = (qi == 0) ? 3 : (qi - 1);  // w,x,y,z -> 3,0,1,2
        data_ptr = waypoint.orientation.coeffs().data() + coeff_idx;
      }
      else if (i < 10)
      {
        data_ptr = waypoint.linear_velocity.data() + (i - 7);
      }
      else if (i < 13)
      {
        data_ptr = waypoint.angular_velocity.data() + (i - 10);
      }
      else if (i < 16)
      {
        data_ptr = waypoint.linear_acceleration.data() + (i - 13);
      }
      else
      {
        data_ptr = waypoint.angular_acceleration.data() + (i - 16);
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(planner_name, interface_name, data_ptr));
    }
  }

  static void read_state_interfaces(
    const std::string& planner_name,
    Waypoint& waypoint,
    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces)
  {
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/pos_x", waypoint.position.x());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/pos_y", waypoint.position.y());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/pos_z", waypoint.position.z());

    double qw, qx, qy, qz;
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/quat_w", qw);
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/quat_x", qx);
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/quat_y", qy);
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/quat_z", qz);
    waypoint.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();

    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/lin_vel_x", waypoint.linear_velocity.x());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/lin_vel_y", waypoint.linear_velocity.y());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/lin_vel_z", waypoint.linear_velocity.z());

    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/ang_vel_x", waypoint.angular_velocity.x());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/ang_vel_y", waypoint.angular_velocity.y());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/ang_vel_z", waypoint.angular_velocity.z());

    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/lin_acc_x", waypoint.linear_acceleration.x());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/lin_acc_y", waypoint.linear_acceleration.y());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/lin_acc_z", waypoint.linear_acceleration.z());

    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/ang_acc_x", waypoint.angular_acceleration.x());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/ang_acc_y", waypoint.angular_acceleration.y());
    base_utils::get_state_interface_value(
        state_interfaces, planner_name + "/" + waypoint.name + "/ang_acc_z", waypoint.angular_acceleration.z());
  }

};  // representation of pose, twist and acceleration

class WaypointPlanner
{
  public:
  WaypointPlanner() = default;

  WaypointPlanner(const std::vector<std::string>& waypoint_names);

  const std::vector<Waypoint>& get_waypoints() const { return waypoints_; }

  std::vector<Waypoint>& get_waypoints() { return waypoints_; }

  const std::vector<std::string>& get_entry_names() { return Waypoint::get_entry_names(); }

  protected:
  std::vector<Waypoint> waypoints_ = {};
};

};  // namespace base_planners

#endif  // BASE_PLANNERS_WAYPOINT_PLANNER_HPP__
