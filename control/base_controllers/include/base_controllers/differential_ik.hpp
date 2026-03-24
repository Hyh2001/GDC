#pragma once
#ifndef DIFFERENTIAL_IK_HPP
#define DIFFERENTIAL_IK_HPP

#include <vector>
#include <stdexcept>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "base_planners/waypoint_planner.hpp"

#include "base_utils/pinocchio_utils.hpp"

namespace base_controllers
{
  using namespace Eigen;
  /*
    A simple differential IK class mimic the differential IK setup of mjlab:
    https://github.com/mujocolab/mjlab/blob/main/src/mjlab/envs/mdp/actions/differential_ik.py
  */

  struct IKWeights
  {
    double position_weight = 1.0;
    double orientation_weight = 1.0;
    double joint_limit_weight = 0.0;
    double posture_weight = 0.0;
    double delta_pos_scale = 1.0;
    double delta_ori_scale = 1.0;
  };

  class DifferentialIK
  {
  public:
    DifferentialIK() = default;
    DifferentialIK(const pinocchio::Model &model,
                   const std::vector<std::string> &joint_names,
                   const std::string &frame_name,
                   const IKWeights &ik_weights = IKWeights(),
                   const VectorXd &q_posture = Eigen::VectorXd(),
                   const double max_dq = 0.5,
                   const double damping = 0.01);
    ~DifferentialIK() = default;

    /*
      absolute position and orientation task.
      @param: waypoint: the desired waypoint to track, which contains position, orientation, velocity and acceleration information. Only position and orientation information is used in this function.
      @param: q: current joint configuration, used for computing the Jacobian and for posture
    */
    VectorXd compute(const base_planners::Waypoint &waypoint,
                 const VectorXd &q);

    /*
      delta position and delta orientation task. delta_ori is represented in axis-angle format, with the rotation axis multiplied by the rotation angle.
      @param: delta_pos: desired change in position of the end-effector
      @param: delta_ori: desired change in orientation of the end-effector, represented in axis-angle format (rotation axis multiplied by rotation angle)
      @param: q: current joint configuration
    */
    VectorXd compute(const VectorXd & delta_pos,
                     const VectorXd & delta_ori,
                     const VectorXd & q);

    /*
      delta position only task.
      @param: delta_pos: desired change in position of the end-effector
      @param: q: current joint configuration
    */
    VectorXd compute(const VectorXd & delta_pos,
                     const VectorXd & q);
  protected:
    pinocchio::Model pin_model_;
    pinocchio::Data pin_data_;
    bool floating_base_ = false; // whether the robot has a floating base, which affects the Jacobian computation
    pinocchio::FrameIndex frame_id_;
    Eigen::VectorXd upper_limits_; // joint position upper limits
    Eigen::VectorXd lower_limits_;
    std::vector<int> perm_;

    // weights
    IKWeights ik_weights_;
    Eigen::VectorXd q_posture_; // desired posture for posture regularization

    double max_dq_ = 0.5; // maximum joint displacement
    double damping_ = 0.01; // damping factor for damped least squares

    std::string frame_name_;
    std::vector<std::string> joint_names_; // active controlled joint names
  };
}


#endif // DIFFERENTIAL_IK_HPP
