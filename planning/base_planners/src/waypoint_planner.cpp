#include "base_planners/waypoint_planner.hpp"

namespace base_planners
{
WaypointPlanner::WaypointPlanner(const std::vector<std::string>& waypoint_names)
{
  waypoints_.resize(waypoint_names.size());
  for (size_t i = 0; i < waypoint_names.size(); ++i)
  {
    waypoints_[i].name = waypoint_names[i];
    waypoints_[i].position = Eigen::Vector3d::Zero();
    waypoints_[i].orientation = Eigen::Quaterniond::Identity();
    waypoints_[i].linear_velocity = Eigen::Vector3d::Zero();
    waypoints_[i].angular_velocity = Eigen::Vector3d::Zero();
    waypoints_[i].linear_acceleration = Eigen::Vector3d::Zero();
    waypoints_[i].angular_acceleration = Eigen::Vector3d::Zero();
  }
}

};  // namespace base_planners
