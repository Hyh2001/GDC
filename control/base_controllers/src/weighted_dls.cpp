#include "base_controllers/weighted_dls.hpp"

#include <algorithm>
#include <stdexcept>

namespace base_controllers
{
  Eigen::VectorXd solve_weighted_dls(const Eigen::MatrixXd &J_pos,
                                     const Eigen::MatrixXd &J_ori,
                                     const Eigen::Vector3d &task_pos,
                                     const Eigen::Vector3d &task_ori,
                                     const Eigen::VectorXd &q_joints,
                                     const Eigen::VectorXd &q_posture,
                                     const WeightedDLSParams &params)
  {
    const double wp2 = params.position_weight * params.position_weight;
    const double wo2 = params.orientation_weight * params.orientation_weight;

    Eigen::MatrixXd JTJ = wp2 * (J_pos.transpose() * J_pos) +
                          wo2 * (J_ori.transpose() * J_ori);
    Eigen::VectorXd JTr = wp2 * (J_pos.transpose() * task_pos) +
                          wo2 * (J_ori.transpose() * task_ori);

    if (params.joint_limit_weight > 0.0)
    {
      if (params.lower_position_limits.size() != q_joints.size() ||
          params.upper_position_limits.size() != q_joints.size())
      {
        throw std::runtime_error("Joint position limit size does not match joint size.");
      }

      Eigen::VectorXd r_limit =
          (params.upper_position_limits - q_joints).cwiseMin(0.0) +
          (params.lower_position_limits - q_joints).cwiseMax(0.0);
      Eigen::VectorXd violated = (r_limit.array() != 0.0).cast<double>();

      const double wl2 = params.joint_limit_weight * params.joint_limit_weight;
      JTJ.diagonal() += wl2 * violated;
      JTr += wl2 * violated.cwiseProduct(r_limit);
    }

    if (params.posture_weight > 0.0)
    {
      if (q_posture.size() != q_joints.size())
      {
        throw std::runtime_error("Posture configuration size does not match joint size.");
      }

      const double wpost2 = params.posture_weight * params.posture_weight;
      JTJ.diagonal().array() += wpost2;
      JTr += wpost2 * (q_posture - q_joints);
    }

    const double lambda = std::max(params.damping, 1e-6);
    JTJ.diagonal().array() += lambda * lambda;

    return JTJ.ldlt().solve(JTr);
  }

  Eigen::VectorXd clamp_joint_velocity(const Eigen::VectorXd &dotq,
                                       const JointVelocityClampParams &params)
  {
    if (dotq.size() != params.lower_limits.size() ||
        dotq.size() != params.upper_limits.size())
    {
      throw std::runtime_error("Velocity limit size does not match joint velocity size.");
    }

    Eigen::VectorXd clamped = dotq.cwiseMax(params.lower_limits)
                                  .cwiseMin(params.upper_limits);
    if (params.max_abs_velocity > 0.0)
    {
      clamped = clamped.cwiseMax(-params.max_abs_velocity)
                       .cwiseMin(params.max_abs_velocity);
    }

    return clamped;
  }
}
