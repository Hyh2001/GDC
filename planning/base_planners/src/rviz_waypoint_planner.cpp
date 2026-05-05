#include "base_planners/rviz_waypoint_planner.hpp"

#include <functional>

namespace base_planners
{

RvizWaypointPlanner::RvizWaypointPlanner(
    const std::string& node_name,
    const std::vector<std::string>& waypoint_names,
    const std::string& fixed_frame)
: fixed_frame_(fixed_frame),
  planner_(waypoint_names)
{
  node_ = std::make_shared<rclcpp::Node>(node_name);

  marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "rviz_waypoint_markers",
      node_);

  executor_.add_node(node_);
}

void RvizWaypointPlanner::initialize()
{
  refresh_markers();
}

void RvizWaypointPlanner::spin_some()
{
  executor_.spin_some();
}

void RvizWaypointPlanner::refresh_markers()
{
  marker_server_->clear();

  for (const auto& waypoint : planner_.get_waypoints())
  {
    insert_marker(waypoint);
  }

  marker_server_->applyChanges();
}

const Waypoint* RvizWaypointPlanner::get_waypoint(const std::string& name) const
{
  for (const auto& waypoint : planner_.get_waypoints())
  {
    if (waypoint.name == name)
    {
      return &waypoint;
    }
  }
  return nullptr;
}

Waypoint* RvizWaypointPlanner::get_waypoint(const std::string& name)
{
  for (auto& waypoint : planner_.get_waypoints())
  {
    if (waypoint.name == name)
    {
      return &waypoint;
    }
  }
  return nullptr;
}

bool RvizWaypointPlanner::set_waypoint_pose(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation)
{
  auto* waypoint = get_waypoint(name);
  if (waypoint == nullptr)
  {
    return false;
  }

  waypoint->position = position;
  waypoint->orientation = orientation.normalized();
  waypoint->linear_velocity.setZero();
  waypoint->angular_velocity.setZero();
  waypoint->linear_acceleration.setZero();
  waypoint->angular_acceleration.setZero();

  refresh_markers();
  return true;
}

visualization_msgs::msg::Marker RvizWaypointPlanner::make_visual_marker() const
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.color.r = 0.15f;
  marker.color.g = 0.80f;
  marker.color.b = 0.25f;
  marker.color.a = 1.0f;
  return marker;
}

visualization_msgs::msg::InteractiveMarker
RvizWaypointPlanner::make_interactive_marker(const Waypoint& waypoint) const
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = fixed_frame_;
  int_marker.name = waypoint.name;
  int_marker.description = waypoint.name;
  int_marker.scale = 0.20;

  int_marker.pose.position.x = waypoint.position.x();
  int_marker.pose.position.y = waypoint.position.y();
  int_marker.pose.position.z = waypoint.position.z();

  int_marker.pose.orientation.w = waypoint.orientation.w();
  int_marker.pose.orientation.x = waypoint.orientation.x();
  int_marker.pose.orientation.y = waypoint.orientation.y();
  int_marker.pose.orientation.z = waypoint.orientation.z();

  visualization_msgs::msg::InteractiveMarkerControl visual_control;
  visual_control.always_visible = true;
  visual_control.markers.push_back(make_visual_marker());
  int_marker.controls.push_back(visual_control);

  auto add_control = [&](double x, double y, double z, const std::string& name, uint8_t mode)
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.name = name;
    control.interaction_mode = mode;
    int_marker.controls.push_back(control);
  };

  add_control(1, 0, 0, "move_x", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
  add_control(0, 1, 0, "move_y", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
  add_control(0, 0, 1, "move_z", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);

  add_control(1, 0, 0, "rot_x", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
  add_control(0, 1, 0, "rot_y", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
  add_control(0, 0, 1, "rot_z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);

  return int_marker;
}

void RvizWaypointPlanner::insert_marker(const Waypoint& waypoint)
{
  auto marker = make_interactive_marker(waypoint);

  marker_server_->insert(
      marker,
      std::bind(&RvizWaypointPlanner::process_feedback, this, std::placeholders::_1));
}

void RvizWaypointPlanner::process_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    return;
  }

  auto* waypoint = get_waypoint(feedback->marker_name);
  if (waypoint == nullptr)
  {
    return;
  }

  waypoint->position.x() = feedback->pose.position.x;
  waypoint->position.y() = feedback->pose.position.y;
  waypoint->position.z() = feedback->pose.position.z;

  waypoint->orientation = Eigen::Quaterniond(
      feedback->pose.orientation.w,
      feedback->pose.orientation.x,
      feedback->pose.orientation.y,
      feedback->pose.orientation.z).normalized();

  waypoint->linear_velocity.setZero();
  waypoint->angular_velocity.setZero();
  waypoint->linear_acceleration.setZero();
  waypoint->angular_acceleration.setZero();

  RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      500,
      "Updated waypoint %s to [%.3f %.3f %.3f]",
      waypoint->name.c_str(),
      waypoint->position.x(),
      waypoint->position.y(),
      waypoint->position.z());

  marker_server_->applyChanges();
}

}  // namespace base_planners
