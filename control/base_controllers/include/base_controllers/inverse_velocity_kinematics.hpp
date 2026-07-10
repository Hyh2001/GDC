#pragma once
#ifndef INVERSE_VELOCITY_KINEMATICS_HPP__
#define INVERSE_VELOCITY_KINEMATICS_HPP__

#include <vector>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "base_utils/pinocchio_utils.hpp"
#include "base_controllers/weighted_dls.hpp"

namespace base_controllers
{
  using namespace Eigen;
  /*
    A simple inverse velocity kinematics class for solving for dotq
    given v = J * dotq where v is the desired end-effector velocity.
    Solving methods:
    1. pseudo-inverse: dotq = J^+ * v
    2. ik + differential (TODO)
  */

  struct Weights
  {
    double position_weight = 1.0;
    double orientation_weight = 1.0;
    double joint_limit_weight = 0.0;
    double posture_weight = 0.0;
    double delta_pos_scale = 1.0;
    double delta_ori_scale = 1.0;
    double max_dq = 0.5;
    double damping = 0.01;
  };


  class InverseVelocityKinematics
  {
  public:
    InverseVelocityKinematics() = default;
    InverseVelocityKinematics(const pinocchio::Model &model,
                              const std::vector<std::string> &joint_names,
                              const std::string &frame_name,
                              const std::string &method = "pseudo_inverse",
                              const Weights &weights = Weights(),
                              const VectorXd &q_posture = Eigen::VectorXd());
    ~InverseVelocityKinematics() = default;

    /*
      @param: v: the desired end-effector twist
      @param: q: current joint configuration, used for computing the Jacobian and for posture regularization
    */
    VectorXd compute(const VectorXd &v,
                     const VectorXd &q,
                     pinocchio::ReferenceFrame ref_frame = pinocchio::WORLD);
  private:
    VectorXd compute_pseudo_inverse(const VectorXd &v,
                                    const VectorXd &q,
                                    pinocchio::ReferenceFrame ref_frame = pinocchio::WORLD);

    VectorXd compute_ik_differential(const VectorXd &v,
                                     const VectorXd &q,
                                     pinocchio::ReferenceFrame ref_frame = pinocchio::WORLD);

    std::string method_{"pseudo_inverse"};

    pinocchio::Model model_;
    pinocchio::Data data_;
    bool floating_base_ = false;
    std::vector<int> perm_{};
    std::vector<std::string> joint_names_{};
    Eigen::VectorXd cw_limits_{}; // joint velocity limits
    Eigen::VectorXd ccw_limits_{};

    Eigen::VectorXd q_posture_{};
    Weights weights_;

    std::string frame_name_{"end_effector"};
    pinocchio::FrameIndex frame_id_{0};
  };
}

#endif // INVERSE_VELOCITY_KINEMATICS_HPP__
