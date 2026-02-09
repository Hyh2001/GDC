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
  std::string name;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};  // representation of pose

class WaypointPlanner
{
  public:
  WaypointPlanner() = default;

  WaypointPlanner(const std::vector<std::string>& waypoint_names);

  const std::vector<Waypoint>& get_waypoints() const { return waypoints_; }

  std::vector<Waypoint>& get_waypoints() { return waypoints_; }

  const std::vector<std::string>& get_entry_names() { return entry_names_; }

  protected:
  std::vector<Waypoint> waypoints_ = {};
  std::vector<std::string> entry_names_ = {"pos_x", "pos_y", "pos_z", "quat_w", "quat_x", "quat_y", "quat_z"};  // wxyz
};

};  // namespace base_planners

#endif  // BASE_PLANNERS_WAYPOINT_PLANNER_HPP__
