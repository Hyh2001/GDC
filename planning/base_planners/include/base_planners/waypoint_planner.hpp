#pragma once
#ifndef BASE_PLANNERS_WAYPOINT_PLANNER_HPP__
#define BASE_PLANNERS_WAYPOINT_PLANNER_HPP__

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "Eigen/Dense"

#include "controller_interface/chainable_controller_interface.hpp"

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

class WaypointPlanner : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  std::vector<Waypoint> waypoints_ = {};
  std::vector<std::string> entry_names_ = {
    "pos_x", "pos_y", "pos_z", "quat_w", "quat_x", "quat_y", "quat_z"
  };  // wxyz
};

};  // namespace base_planners

#endif  // BASE_PLANNERS_WAYPOINT_PLANNER_HPP__
