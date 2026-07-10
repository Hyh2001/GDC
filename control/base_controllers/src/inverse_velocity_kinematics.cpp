#include "base_controllers/inverse_velocity_kinematics.hpp"
#include "base_controllers/weighted_dls.hpp"

#include <algorithm>
#include <stdexcept>

namespace base_controllers
{

  InverseVelocityKinematics::InverseVelocityKinematics(const pinocchio::Model &model,
                                                         const std::vector<std::string> &joint_names,
                                                         const std::string &frame_name,
                                                         const std::string &method,
                                                         const Weights &weights,
                                                         const VectorXd &q_posture)
  : method_(method),
    model_(model),
    data_(model_),
    floating_base_(pinocchio_utils::is_floating_base(model_)),
    perm_(pinocchio_utils::build_joint_reorder_map(joint_names, model_)),
    joint_names_(joint_names),
    q_posture_(q_posture),
    weights_(weights),
    frame_name_(frame_name)
  {
    // check posture size
    if (q_posture_.size() != static_cast<Eigen::Index>(joint_names_.size()) && weights_.posture_weight > 0.0)
    {
      throw std::runtime_error("Posture weight is positive but posture configuration size does not match joint names size.");
    }
    // check frame existence
    if (model_.existFrame(frame_name_))
    {
      frame_id_ = model_.getFrameId(frame_name_);
    }
    else
    {
      throw std::runtime_error("Frame name does not exist in the Pinocchio model: " + frame_name_);
    }

    cw_limits_.resize(static_cast<Eigen::Index>(joint_names_.size()));
    ccw_limits_.resize(static_cast<Eigen::Index>(joint_names_.size()));
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      const auto joint_id = model_.getJointId(joint_names_[i]);
      const Eigen::Index idx_v = model_.joints[joint_id].idx_v();
      cw_limits_(static_cast<Eigen::Index>(i)) = model_.velocityLimit(idx_v);
      ccw_limits_(static_cast<Eigen::Index>(i)) = -model_.velocityLimit(idx_v);
    }
  }

  VectorXd InverseVelocityKinematics::compute(const VectorXd &v,
                                              const VectorXd &q,
                                              pinocchio::ReferenceFrame ref_frame)
  {
    // check size
    if (v.size() != 6 || q.size() != model_.nq)
    {
      throw std::runtime_error("Input size mismatch: v should be of size 6 and q should match the model state variable size.");
    }

    // call correspondng method
    if (method_ == "pseudo_inverse")
    {
      return compute_pseudo_inverse(v, q, ref_frame);
    }
    else if (method_ == "ik_differential")
    {
      return compute_ik_differential(v, q, ref_frame);
    }
    else
    {
      throw std::runtime_error("Unknown method: " + method_ + " for solving inverse velocity kinematics.");
    }
  }

  VectorXd InverseVelocityKinematics::compute_pseudo_inverse(const VectorXd &v,
                                                             const VectorXd &q,
                                                             pinocchio::ReferenceFrame ref_frame)
  {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model_, data_, q, frame_id_, ref_frame, J);

    Eigen::MatrixXd J_joints(6, static_cast<Eigen::Index>(joint_names_.size()));
    Eigen::VectorXd q_joints(static_cast<Eigen::Index>(joint_names_.size()));
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      const auto joint_id = model_.getJointId(joint_names_[i]);
      const Eigen::Index idx_v = model_.joints[joint_id].idx_v();
      const Eigen::Index idx_q = model_.joints[joint_id].idx_q();
      const Eigen::Index out_idx = static_cast<Eigen::Index>(i);

      J_joints.col(out_idx) = J.col(idx_v);
      q_joints(out_idx) = q(idx_q);
    }

    WeightedDLSParams dls_params;
    dls_params.position_weight = weights_.position_weight;
    dls_params.orientation_weight = weights_.orientation_weight;
    dls_params.joint_limit_weight = 0.0;
    dls_params.posture_weight = weights_.posture_weight;
    dls_params.damping = weights_.damping;

    Eigen::VectorXd dotq = solve_weighted_dls(
        J_joints.topRows<3>(),
        J_joints.bottomRows<3>(),
        v.head<3>(),
        v.tail<3>(),
        q_joints,
        q_posture_,
        dls_params);

    JointVelocityClampParams clamp_params;
    clamp_params.lower_limits = ccw_limits_;
    clamp_params.upper_limits = cw_limits_;
    clamp_params.max_abs_velocity = weights_.max_dq;

    return clamp_joint_velocity(dotq, clamp_params);
  }

  VectorXd InverseVelocityKinematics::compute_ik_differential(const VectorXd &v,
                                                              const VectorXd &q,
                                                              pinocchio::ReferenceFrame ref_frame)
  {
    (void)v;
    (void)q;
    (void)ref_frame;
    throw std::runtime_error("ik_differential method is not implemented yet.");
  }

}; //namespace base_controllers
