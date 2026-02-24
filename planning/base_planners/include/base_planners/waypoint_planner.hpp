#pragma once
#ifndef BASE_PLANNERS_WAYPOINT_PLANNER_HPP__
#define BASE_PLANNERS_WAYPOINT_PLANNER_HPP__

#include <vector>

#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

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

  // Static helper for full state representation (19 entries)
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
