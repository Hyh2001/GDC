#include "base_controllers/differential_ik.hpp"
#include "base_controllers/weighted_dls.hpp"

namespace base_controllers
{
  DifferentialIK::DifferentialIK(const pinocchio::Model &model,
                                 const std::vector<std::string> &joint_names,
                                 const std::string &frame_name,
                                 const IKWeights &ik_weights,
                                 const VectorXd &q_posture,
                                 const double max_dq,
                                 const double damping)
  : pin_model_(model),
    pin_data_(pin_model_),
    floating_base_(pinocchio_utils::is_floating_base(pin_model_)),
    perm_(pinocchio_utils::build_joint_reorder_map(joint_names, pin_model_)),
    ik_weights_(ik_weights),
    q_posture_(q_posture),
    max_dq_(max_dq),
    damping_(std::max(damping, 1e-6)),
    frame_name_(frame_name),
    joint_names_(joint_names)
  {
    if (q_posture_.size() != static_cast<Eigen::Index>(joint_names_.size()) && ik_weights_.posture_weight > 0.0)
    {
      throw std::runtime_error("Posture weight is positive but posture configuration size does not match joint names size.");
    }

    if (pin_model_.existFrame(frame_name_))
    {
      frame_id_ = pin_model_.getFrameId(frame_name_);
    }
    else
    {
      throw std::runtime_error("Frame name does not exist in the Pinocchio model: " + frame_name_);
    }

    VectorXd upper_limits = pin_model_.upperPositionLimit;
    VectorXd lower_limits = pin_model_.lowerPositionLimit;
    if (floating_base_)
    {
      upper_limits_ = pinocchio_utils::reorder_pinocchio_to_joint(
          upper_limits.tail(upper_limits.size() - 7), perm_);
      lower_limits_ = pinocchio_utils::reorder_pinocchio_to_joint(
          lower_limits.tail(lower_limits.size() - 7), perm_);
    }
    else
    {
      upper_limits_ = pinocchio_utils::reorder_pinocchio_to_joint(upper_limits, perm_);
      lower_limits_ = pinocchio_utils::reorder_pinocchio_to_joint(lower_limits, perm_);
    }
  }

  VectorXd DifferentialIK::compute(const base_planners::Waypoint &waypoint,
                                   const VectorXd &q)
  {
    pinocchio::forwardKinematics(pin_model_, pin_data_, q);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);
    pinocchio::SE3 current_ee_pose = pin_data_.oMf[frame_id_];

    // error: target - current
    Vector3d pos_error = waypoint.position - current_ee_pose.translation();
    Eigen::Matrix3d target_rotation = waypoint.orientation.toRotationMatrix();
    Eigen::Matrix3d current_rotation = current_ee_pose.rotation();
    Eigen::Matrix3d rotation_error_matrix = target_rotation * current_rotation.transpose();
    Eigen::AngleAxisd rotation_error_aa(rotation_error_matrix);
    Vector3d ori_error = rotation_error_aa.axis() * rotation_error_aa.angle();

    // jacobian
    pinocchio::Data::Matrix6x J(6, pin_model_.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(pin_model_, pin_data_, q, frame_id_, pinocchio::WORLD, J);
    Eigen::MatrixXd J_pos = J.topRows<3>();  // position jacobian (3 x nv)
    Eigen::MatrixXd J_ori = J.bottomRows<3>();  // orientation jacobian (3 x nv)

    //////////////// damped least square
    // reorder jacobians
    Eigen::MatrixXd J_pos_compact = J_pos.rightCols(joint_names_.size());
    Eigen::MatrixXd J_ori_compact = J_ori.rightCols(joint_names_.size());
    Eigen::MatrixXd J_pos_joints(3, joint_names_.size());
    Eigen::MatrixXd J_ori_joints(3, joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      J_pos_joints.col(i) = J_pos_compact.col(perm_[i]);
      J_ori_joints.col(i) = J_ori_compact.col(perm_[i]);
    }

    int q_joint_offset = floating_base_ ? 7 : 0;
    Eigen::VectorXd q_joints(static_cast<Eigen::Index>(joint_names_.size()));
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      q_joints(static_cast<Eigen::Index>(i)) = q(q_joint_offset + perm_[i]);
    }

    WeightedDLSParams dls_params;
    dls_params.position_weight = ik_weights_.position_weight;
    dls_params.orientation_weight = ik_weights_.orientation_weight;
    dls_params.joint_limit_weight = ik_weights_.joint_limit_weight;
    dls_params.posture_weight = ik_weights_.posture_weight;
    dls_params.damping = damping_;
    dls_params.lower_position_limits = lower_limits_;
    dls_params.upper_position_limits = upper_limits_;

    Eigen::VectorXd dq = solve_weighted_dls(
        J_pos_joints,
        J_ori_joints,
        pos_error,
        ori_error,
        q_joints,
        q_posture_,
        dls_params);

    JointVelocityClampParams clamp_params;
    clamp_params.lower_limits = Eigen::VectorXd::Constant(dq.size(), -max_dq_);
    clamp_params.upper_limits = Eigen::VectorXd::Constant(dq.size(), max_dq_);
    clamp_params.max_abs_velocity = 0.0;

    return clamp_joint_velocity(dq, clamp_params);
  }

  VectorXd DifferentialIK::compute(const VectorXd & delta_pos,
                                   const VectorXd & delta_ori,
                                   const VectorXd & q)
  {
    // compute forward kinematics to get current pose
    pinocchio::forwardKinematics(pin_model_, pin_data_, q);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);
    pinocchio::SE3 current_ee_pose = pin_data_.oMf[frame_id_];

    // create target waypoint from current pose + delta
    base_planners::Waypoint target_waypoint;
    target_waypoint.position = current_ee_pose.translation() + delta_pos;

    // apply orientation delta using angle-axis
    Eigen::AngleAxisd delta_rotation(delta_ori.norm(), delta_ori.normalized());
    Eigen::Matrix3d target_rotation = delta_rotation.toRotationMatrix() * current_ee_pose.rotation();
    target_waypoint.orientation = Eigen::Quaterniond(target_rotation);

    return compute(target_waypoint, q);
  }

  VectorXd DifferentialIK::compute(const VectorXd & delta_pos,
                                   const VectorXd & q)
  {
    // compute forward kinematics to get current pose
    pinocchio::forwardKinematics(pin_model_, pin_data_, q);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);
    pinocchio::SE3 current_ee_pose = pin_data_.oMf[frame_id_];

    // create target waypoint from current pose + delta
    base_planners::Waypoint target_waypoint;
    target_waypoint.position = current_ee_pose.translation() + delta_pos;
    target_waypoint.orientation = Eigen::Quaterniond(current_ee_pose.rotation());

    return compute(target_waypoint, q);
  }

}; // namespace base_controllers
