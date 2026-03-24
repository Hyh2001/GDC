#include "base_controllers/differential_ik.hpp"

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
    if (q_posture_.size() != joint_names_.size() && ik_weights_.posture_weight > 0.0)
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
    int joint_offset = floating_base_ ? 6 : 0;  // 6 DOF for floating base velocity
    Eigen::MatrixXd J_pos_compact = J_pos.rightCols(joint_names_.size());
    Eigen::MatrixXd J_ori_compact = J_ori.rightCols(joint_names_.size());
    Eigen::MatrixXd J_pos_joints(3, joint_names_.size());
    Eigen::MatrixXd J_ori_joints(3, joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      J_pos_joints.col(i) = J_pos_compact.col(perm_[i]);
      J_ori_joints.col(i) = J_ori_compact.col(perm_[i]);
    }

    // weighted JTJ, JTdx
    double wp2 = ik_weights_.position_weight * ik_weights_.position_weight;
    double wo2 = ik_weights_.orientation_weight * ik_weights_.orientation_weight;

    Eigen::MatrixXd JTJ = wp2 * (J_pos_joints.transpose() * J_pos_joints) +
                          wo2 * (J_ori_joints.transpose() * J_ori_joints);
    Eigen::VectorXd JTdx = wp2 * (J_pos_joints.transpose() * pos_error) +
                           wo2 * (J_ori_joints.transpose() * ori_error);

    // joint limit penalty
    int q_joint_offset = floating_base_ ? 7 : 0;
    Eigen::VectorXd q_joints(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      q_joints(i) = q(q_joint_offset + perm_[i]);
    }
    Eigen::VectorXd r_limit = (upper_limits_ - q_joints).cwiseMin(0.0) +
                              (lower_limits_ - q_joints).cwiseMax(0.0);
    Eigen::VectorXd violated = (r_limit.array() != 0.0).cast<double>();
    double wl2 = ik_weights_.joint_limit_weight * ik_weights_.joint_limit_weight;
    JTJ.diagonal() += wl2 * violated;
    JTdx += wl2 * violated.cwiseProduct(r_limit);

    // posture regularization
    if (ik_weights_.posture_weight > 0.0){
      Eigen::VectorXd r_posture = q_posture_ - q_joints;
      double wpost2 = ik_weights_.posture_weight * ik_weights_.posture_weight;
      JTJ.diagonal().array() += wpost2;
      JTdx += wpost2 * r_posture;
    }

    // damping
    JTJ.diagonal().array() += damping_ * damping_;

    // solve and clamp
    Eigen::VectorXd dq = JTJ.ldlt().solve(JTdx);
    dq = dq.cwiseMax(-max_dq_).cwiseMin(max_dq_);

    return dq;
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
