#include "base_planners/trajectory_planner.hpp"

#include <algorithm>

namespace base_planners
{

LinePoseTrajectory::LinePoseTrajectory(const Params& params)
{
  set_params(params);
}

void LinePoseTrajectory::set_params(const Params& params)
{
  if (params.duration <= 0.0)
  {
    throw std::invalid_argument("LinePoseTrajectory duration must be positive.");
  }
  if (params.orientation.norm() <= 1e-9)
  {
    throw std::invalid_argument("LinePoseTrajectory orientation quaternion norm is too small.");
  }

  params_ = params;
  params_.orientation.normalize();
}

const LinePoseTrajectory::Params& LinePoseTrajectory::params() const
{
  return params_;
}

void LinePoseTrajectory::reset(double time)
{
  start_time_ = time;
}

Waypoint LinePoseTrajectory::sample(double time) const
{
  const double elapsed_time = std::clamp(time - start_time_, 0.0, params_.duration);
  const double normalized_time = elapsed_time / params_.duration;

  // Quintic minimum-jerk time scaling with zero velocity and acceleration
  // at the start and target.
  const double u2 = normalized_time * normalized_time;
  const double u3 = u2 * normalized_time;
  const double u4 = u3 * normalized_time;
  const double u5 = u4 * normalized_time;
  const double position_scale = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  const double velocity_scale = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / params_.duration;
  const double acceleration_scale =
      (60.0 * normalized_time - 180.0 * u2 + 120.0 * u3) / (params_.duration * params_.duration);
  const Eigen::Vector3d displacement = params_.target - params_.center;

  Waypoint waypoint;
  waypoint.position = params_.center + position_scale * displacement;
  waypoint.orientation = params_.orientation;
  waypoint.linear_velocity = velocity_scale * displacement;
  waypoint.linear_acceleration = acceleration_scale * displacement;
  waypoint.angular_velocity.setZero();
  waypoint.angular_acceleration.setZero();
  return waypoint;
}

CirclePoseTrajectory::CirclePoseTrajectory(const Params& params)
{
  set_params(params);
}

void CirclePoseTrajectory::set_params(const Params& params)
{
  params_ = params;
  if (params_.normal.norm() > 1e-9)
  {
    params_.normal.normalize();
  }
  else
  {
    params_.normal = Eigen::Vector3d::UnitZ();
  }

  params_.radius_direction -= params_.radius_direction.dot(params_.normal) * params_.normal;
  if (params_.radius_direction.norm() > 1e-9)
  {
    params_.radius_direction.normalize();
  }
  else
  {
    params_.radius_direction = params_.normal.unitOrthogonal();
  }
  params_.orientation.normalize();
}

const CirclePoseTrajectory::Params& CirclePoseTrajectory::params() const
{
  return params_;
}

void CirclePoseTrajectory::reset(double time)
{
  start_time_ = time;
}

Waypoint CirclePoseTrajectory::sample(double time) const
{
  const double elapsed_time = time - start_time_;
  const double theta = params_.phase + params_.omega * elapsed_time;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  const Eigen::Vector3d tangent_direction = params_.normal.cross(params_.radius_direction);

  const Eigen::Vector3d radial = cos_theta * params_.radius_direction + sin_theta * tangent_direction;
  const Eigen::Vector3d tangent = -sin_theta * params_.radius_direction + cos_theta * tangent_direction;

  Waypoint waypoint;
  waypoint.position = params_.center + params_.radius * radial;
  waypoint.orientation = params_.orientation;
  waypoint.linear_velocity = params_.radius * params_.omega * tangent;
  waypoint.linear_acceleration = -params_.radius * params_.omega * params_.omega * radial;
  waypoint.angular_velocity.setZero();
  waypoint.angular_acceleration.setZero();
  return waypoint;
}

SinePoseTrajectory::SinePoseTrajectory(const Params& params)
{
  set_params(params);
}

void SinePoseTrajectory::set_params(const Params& params)
{
  params_ = params;
  if (params_.direction.norm() > 1e-9)
  {
    params_.direction.normalize();
  }
  else
  {
    params_.direction = Eigen::Vector3d::UnitZ();
  }
  params_.orientation.normalize();
}

const SinePoseTrajectory::Params& SinePoseTrajectory::params() const
{
  return params_;
}

void SinePoseTrajectory::reset(double time)
{
  start_time_ = time;
}

Waypoint SinePoseTrajectory::sample(double time) const
{
  const double elapsed_time = time - start_time_;
  const double theta = params_.phase + params_.omega * elapsed_time;
  const double displacement = params_.amplitude * std::sin(theta);
  const double velocity = params_.amplitude * params_.omega * std::cos(theta);
  const double acceleration = -params_.amplitude * params_.omega * params_.omega * std::sin(theta);

  Waypoint waypoint;
  waypoint.position = params_.center + displacement * params_.direction;
  waypoint.orientation = params_.orientation;
  waypoint.linear_velocity = velocity * params_.direction;
  waypoint.linear_acceleration = acceleration * params_.direction;
  waypoint.angular_velocity.setZero();
  waypoint.angular_acceleration.setZero();
  return waypoint;
}

PoseTrajectoryPlanner::PoseTrajectoryPlanner() = default;

PoseTrajectoryPlanner::PoseTrajectoryPlanner(const std::vector<std::string>& waypoint_names,
                                             std::vector<std::unique_ptr<PoseTrajectory>> trajectories)
    : WaypointPlanner(waypoint_names), trajectories_(std::move(trajectories))
{
  if (waypoints_.size() != trajectories_.size())
  {
    throw std::invalid_argument("PoseTrajectoryPlanner requires one trajectory per waypoint.");
  }
  for (const auto& trajectory : trajectories_)
  {
    if (trajectory == nullptr)
    {
      throw std::invalid_argument("PoseTrajectoryPlanner requires non-null trajectories.");
    }
  }
}

void PoseTrajectoryPlanner::reset(double time)
{
  elapsed_time_ = time;

  for (auto& trajectory : trajectories_)
  {
    trajectory->reset(time);
  }

  update(0.0);
}

void PoseTrajectoryPlanner::update(double dt)
{
  elapsed_time_ += dt;

  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    Waypoint sampled = trajectories_[i]->sample(elapsed_time_);

    sampled.name = waypoints_[i].name;
    waypoints_[i] = sampled;
  }
}

const std::vector<Waypoint>& PoseTrajectoryPlanner::get_waypoints() const
{
  return WaypointPlanner::get_waypoints();
}

std::vector<Waypoint>& PoseTrajectoryPlanner::get_waypoints()
{
  return WaypointPlanner::get_waypoints();
}

const Waypoint& PoseTrajectoryPlanner::get_waypoint(size_t index) const
{
  return waypoints_.at(index);
}

Waypoint& PoseTrajectoryPlanner::get_waypoint(size_t index)
{
  return waypoints_.at(index);
}

void PoseTrajectoryPlanner::set_trajectory(size_t index, std::unique_ptr<PoseTrajectory> trajectory)
{
  if (index >= trajectories_.size())
  {
    throw std::out_of_range("Trajectory index out of range.");
  }
  if (trajectory == nullptr)
  {
    throw std::invalid_argument("Cannot set null trajectory.");
  }

  trajectories_[index] = std::move(trajectory);
  trajectories_[index]->reset(elapsed_time_);
}

}  // namespace base_planners
