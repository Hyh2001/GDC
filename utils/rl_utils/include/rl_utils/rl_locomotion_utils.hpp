#pragma once
#ifndef RL_UTILS_RL_LOCOMOTION_UTILS_HPP
#define RL_UTILS_RL_LOCOMOTION_UTILS_HPP

#include <vector>

#include "Eigen/Dense"

namespace rl_utils
{

inline void projected_gravity(const Eigen::Quaterniond& base_w, Eigen::Vector3d& gravity_proj)
{
  /*
      Projects the gravity vector into the base frame
  */
  Eigen::Vector3d gravity_w(0.0, 0.0, -1);
  gravity_proj = base_w.inverse() * gravity_w;
}

};  // namespace rl_utils

#endif  // RL_UTILS_RL_LOCOMOTION_UTILS_HPP
