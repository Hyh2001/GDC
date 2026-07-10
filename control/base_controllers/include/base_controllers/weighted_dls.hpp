#pragma once
#ifndef WEIGHTED_DLS_HPP__
#define WEIGHTED_DLS_HPP__

#include <Eigen/Dense>

namespace base_controllers
{
  struct WeightedDLSParams
  {
    double position_weight = 1.0;
    double orientation_weight = 1.0;
    double joint_limit_weight = 0.0;
    double posture_weight = 0.0;
    double damping = 0.01;
    Eigen::VectorXd lower_position_limits{};
    Eigen::VectorXd upper_position_limits{};
  };

  struct JointVelocityClampParams
  {
    Eigen::VectorXd lower_limits{};
    Eigen::VectorXd upper_limits{};
    double max_abs_velocity = 0.0;
  };

  Eigen::VectorXd solve_weighted_dls(const Eigen::MatrixXd &J_pos,
                                     const Eigen::MatrixXd &J_ori,
                                     const Eigen::Vector3d &task_pos,
                                     const Eigen::Vector3d &task_ori,
                                     const Eigen::VectorXd &q_joints,
                                     const Eigen::VectorXd &q_posture,
                                     const WeightedDLSParams &params);

  Eigen::VectorXd clamp_joint_velocity(const Eigen::VectorXd &dotq,
                                       const JointVelocityClampParams &params);
}

#endif // WEIGHTED_DLS_HPP__
