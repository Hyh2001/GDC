#pragma once
#ifndef BASE_PLANNERS_VELOCITY_PLANNER_HPP__
#define BASE_PLANNERS_VELOCITY_PLANNER_HPP__

#include <vector>

#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"

namespace base_planners
{
/*
    VelocityPlanner implements a velocity planner taking velocity commands and
    applying the commands.
*/

class VelocityPlanner
{
  public:
  VelocityPlanner() = default;

  VelocityPlanner(std::array<double, 3> max_velocity) : max_velocity_(max_velocity){};

  std::array<double,3> compute(std::array<double, 3> velocity_cmd_raw);

  const std::array<double,3>& get_velocity_cmd() const { return velocity_cmd_; }

  std::array<double,3> get_velocity_cmd() { return velocity_cmd_;}

  protected:
  std::array<double, 3> max_velocity_{1.0, 1.0, 0.5};  // vx, vy, yaw rate
  std::array<double, 3> velocity_cmd_{0.0, 0.0, 0.0};  // vx, vy, yaw rate
};

};  // namespace base_planners

#endif  // BASE_PLANNERS_VELOCITY_PLANNER_HPP__
