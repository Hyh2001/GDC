/*
 This file implements the normally used observation functions
*/
#pragma once
#ifndef OBSERVATIONS_HPP_
#define OBSERVATIONS_HPP_

#include "Eigen/Dense"

#include "rl_utils/mdp/observation/observation_manager.hpp"

namespace observation
{
// passthrough
inline std::vector<double> passthrough(std::vector<double> values)
{
  return values;
}

// passthrough for Eigen vector
inline std::vector<double> passthrough(const Eigen::VectorXd& values)
{
  return std::vector<double>(values.data(), values.data() + values.size());
}

// projected gravity
inline std::vector<double> projected_gravity(const Eigen::Quaterniond& base_w)
{
  /*
      Projects the gravity vector into the base frame
  */
  Eigen::Vector3d gravity_w(0.0, 0.0, -1);
  Eigen::Vector3d gravity_proj = base_w.inverse() * gravity_w;
  return std::vector<double>(gravity_proj.data(), gravity_proj.data() + gravity_proj.size());
}

// joint values
inline std::vector<double> joint_values(const std::vector<double>& joint_values, const std::vector<double>& joint_inits)
{
  std::vector<double> joint_obs(joint_values.size());
  for (size_t i = 0; i < joint_values.size(); ++i)  {
    joint_obs[i] = joint_values[i] - joint_inits[i];
  }
  return joint_obs;
}


}




#endif // OBSERVATIONS_HPP_
