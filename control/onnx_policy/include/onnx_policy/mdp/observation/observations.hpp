#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "onnx_policy/mdp/observation/observation_manager.hpp"

namespace onnx_policy::observation
{
inline std::vector<double> passthrough(std::vector<double> values)
{
  return values;
}

inline std::vector<double> passthrough(const Eigen::VectorXd& values)
{
  if (values.size() == 0)
    return {};
  return {values.data(), values.data() + values.size()};
}

inline std::vector<double> projected_gravity(const Eigen::Quaterniond& orientation)
{
  const double norm = orientation.norm();
  if (!orientation.coeffs().allFinite() || !std::isfinite(norm) || norm <= 1e-12)
    throw std::invalid_argument("Projected gravity requires a nonzero finite quaternion");
  const Eigen::Vector3d gravity = orientation.normalized().conjugate() * Eigen::Vector3d(0, 0, -1);
  return {gravity.x(), gravity.y(), gravity.z()};
}

inline std::vector<double> joint_values(const std::vector<double>& values, const std::vector<double>& offsets)
{
  if (values.size() != offsets.size())
    throw std::invalid_argument("Joint offset size mismatch");
  std::vector<double> result(values.size());
  for (size_t i = 0; i < values.size(); ++i)
  {
    if (!std::isfinite(values[i]) || !std::isfinite(offsets[i]))
      throw std::invalid_argument("Nonfinite joint value or offset");
    result[i] = values[i] - offsets[i];
    if (!std::isfinite(result[i]))
      throw std::invalid_argument("Joint offset overflow");
  }
  return result;
}
}  // namespace onnx_policy::observation
