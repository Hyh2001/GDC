#pragma once
#ifndef RVIZ_WAYPOINT_PLANNER_HPP__
#define RVIZ_WAYPOINT_PLANNER_HPP__

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "base_planners/waypoint_planner.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace base_planners
{

class RvizWaypointPlanner
{
public:
  RvizWaypointPlanner(
      const std::string& node_name,
      const std::vector<std::string>& waypoint_names,
      const std::string& fixed_frame = "base_link");

  rclcpp::Node::SharedPtr get_node() const { return node_; }

  void initialize();
  void spin_some();
  void refresh_markers();

  const std::vector<Waypoint>& get_waypoints() const { return planner_.get_waypoints(); }
  std::vector<Waypoint>& get_waypoints() { return planner_.get_waypoints(); }

  const Waypoint* get_waypoint(const std::string& name) const;
  Waypoint* get_waypoint(const std::string& name);

  bool set_waypoint_pose(
      const std::string& name,
      const Eigen::Vector3d& position,
      const Eigen::Quaterniond& orientation);

private:
  visualization_msgs::msg::Marker make_visual_marker() const;
  visualization_msgs::msg::InteractiveMarker make_interactive_marker(const Waypoint& waypoint) const;

  void insert_marker(const Waypoint& waypoint);
  void process_feedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  std::string fixed_frame_;
  WaypointPlanner planner_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

  // velocity target calculation
  std::unordered_map<std::string, Eigen::Vector3d> previous_marker_positions_{};
  std::unordered_map<std::string, Eigen::Quaterniond> previous_marker_orientations_{};
  std::unordered_map<std::string, rclcpp::Time> previous_marker_update_times_{};
};

}  // namespace base_planners

#endif  // RVIZ_WAYPOINT_PLANNER_HPP__
