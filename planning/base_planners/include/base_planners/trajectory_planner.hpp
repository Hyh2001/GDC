#pragma once
#ifndef TRAJETORY_PLANNER_HPP__
#define TRAJETORY_PLANNER_HPP__

#include <memory>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "base_planners/waypoint_planner.hpp"

namespace base_planners
{
/*
  virtual trajectory class
*/
class PoseTrajectory
{
public:
  virtual ~PoseTrajectory() = default;

  virtual void reset(double time = 0.0) = 0;

  virtual base_planners::Waypoint sample(double time) const = 0;
};

// line
class LinePoseTrajectory : public PoseTrajectory
{
public:
  struct Params
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    double amplitude = 0.0;
    double omega = 0.0;
    double phase = 0.0;
  };

  explicit LinePoseTrajectory(const Params& params);

  void set_params(const Params& params);

  const Params& params() const;

  void reset(double time = 0.0) override;

  base_planners::Waypoint sample(double time) const override;

private:
  Params params_{};
  double start_time_ = 0.0;
};

//circle
class CirclePoseTrajectory : public PoseTrajectory
{
public:
  struct Params
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d radius_direction = Eigen::Vector3d::UnitX();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    double radius = 0.0;
    double omega = 0.0;
    double phase = 0.0;
  };

  explicit CirclePoseTrajectory(const Params& params);

  void set_params(const Params& params);

  const Params& params() const;

  void reset(double time = 0.0) override;

  base_planners::Waypoint sample(double time) const override;

private:
  Params params_{};
  double start_time_ = 0.0;
};

//sine
class SinePoseTrajectory : public PoseTrajectory
{
public:
  struct Params
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    double amplitude = 0.0;
    double omega = 0.0;
    double phase = 0.0;
  };

  explicit SinePoseTrajectory(const Params& params);

  void set_params(const Params& params);

  const Params& params() const;

  void reset(double time = 0.0) override;

  base_planners::Waypoint sample(double time) const override;

private:
  Params params_{};
  double start_time_ = 0.0;
};


/*
  trajectory planner
*/
class PoseTrajectoryPlanner : public WaypointPlanner
{
public:
  PoseTrajectoryPlanner();

  PoseTrajectoryPlanner(
      const std::vector<std::string>& waypoint_names,
      std::vector<std::unique_ptr<PoseTrajectory>> trajectories);

  void reset(double time = 0.0);

  void update(double dt);

  const std::vector<Waypoint>& get_waypoints() const;

  std::vector<Waypoint>& get_waypoints();

  const Waypoint& get_waypoint(size_t index) const;

  Waypoint& get_waypoint(size_t index);

  void set_trajectory(size_t index, std::unique_ptr<PoseTrajectory> trajectory);

private:
  std::vector<std::unique_ptr<PoseTrajectory>> trajectories_;
  double elapsed_time_ = 0.0;
};

} // namespace base_planners


#endif // TRAJETORY_PLANNER_HPP__
