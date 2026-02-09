#include "base_planners/velocity_planner.hpp"

namespace base_planners
{
std::array<double,3> VelocityPlanner::compute(std::array<double, 3> velocity_cmd_raw)
{
  // clip the velocity commands
  for (size_t i = 0; i < 3; ++i)
  {
    velocity_cmd_[i] = velocity_cmd_raw[i];
    if (velocity_cmd_raw[i] > max_velocity_[i])
    {
      velocity_cmd_[i] = max_velocity_[i];
    }
    else if (velocity_cmd_raw[i] < -max_velocity_[i])
    {
      velocity_cmd_[i] = -max_velocity_[i];
    }
  }

  return velocity_cmd_;
}

};  // namespace base_planners
