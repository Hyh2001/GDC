#pragma once
#ifndef IEKF_HPP
#define IEKF_HPP

#include <thread>
#include <chrono>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "IEKF/InEKF.h"
#include "MOB.hpp"

using namespace Eigen;
using namespace inekf;

// ============================================================================
// Parameter Structure
// ============================================================================
struct robot_params
{
    // ------------------- Priors -------------------
    std::vector<double> p_init_std_;
    std::vector<double> v_init_std_;
    double orientation_init_std_;
    double foot_init_std_;
    double accel_bias_init_std_;
    double angular_bias_init_std_;

    // ------------------- Process Model -------------------
    std::vector<double> p_process_std_;
    std::vector<double> accel_input_std_;
    std::vector<double> gyro_input_std_;
    std::vector<double> accel_bias_process_std_;
    std::vector<double> angular_bias_process_std_;

    std::vector<double> q_body_imu_;
    std::vector<double> p_body_imu_;

    // ------------------- Measurement Model -------------------
    std::vector<double> joint_position_std_;
    std::vector<double> joint_velocity_std_;
    std::vector<double> foot_slide_std_;
    std::vector<double> foot_swing_std_;
    double contact_theshold_;

    // ------------------- Estimator Settings -------------------
    int rate_;
    int N_;
    int num_legs_;
    int dim_legs_;
    int using_lo_p_;
    int using_lo_v_;
    int using_mob_;
    int visualize_;
    std::string log_name_;
    std::string robot_name_;
    std::string urdf_path_;
    std::vector<std::string> contact_names_;
    std::vector<std::string> joint_names_;
};

// ============================================================================
// Sensor Store
// ============================================================================
struct robot_store
{
    // IMU
    double imu_time_;
    Vector3d accel_b_;
    Vector3d omega_b_;

    // Encoder & Contact
    VectorXd joint_states_position_;
    VectorXd joint_states_velocity_;
    VectorXd joint_states_effort_;
    VectorXd contact_;

    // Ground Truth / External
    Quaterniond quaternion_;
    Vector3d p_;
    Vector3d v_b_;
};

// ============================================================================
// IEKF Class
// ============================================================================
class robot_IEKF
{
public:
    // ------------------- Constructor -------------------
    robot_IEKF(pinocchio::Model &pin_model,
               pinocchio::Data &pin_data,
               std::shared_ptr<robot_store> store,
               std::shared_ptr<robot_params> params);

    // ------------------- Public API -------------------
    void initialize();
    void update(int T);
    void reset();
    bool isinit = false;

public:
    // ------------------- Public State -------------------
    VectorXd x_est_;
    Vector3d rpy_est_ = Vector3d::Zero();
    Vector3d v_b_est_ = Vector3d::Zero(); // Body velocity in the body frame

    Vector3d accel_b_;
    Vector3d omega_b_;
    VectorXd contact_;
    VectorXd contact_torque_;

    // Ground truth orientation from AHRS (IMU) for initialization and comparison
    Matrix3d R_gt_ = Matrix3d::Identity();
    Quaterniond quat_gt_ = Quaterniond::Identity();
    VectorXd q_gt = VectorXd::Zero(4); // w, x, y, z
    Vector3d rpy_gt_ = Vector3d::Zero();

private:
    // ========================================================================
    // External Dependencies
    // ========================================================================
    pinocchio::Model &pin_model_;
    pinocchio::Data &pin_data_;

    std::shared_ptr<robot_store> store_ptr_;
    std::shared_ptr<robot_params> params_ptr_;

    InEKF filter;
    MomentumObserver *mob;

private:
    // ========================================================================
    // Core Estimator Parameters
    // ========================================================================
    double dt_;
    int N_;
    double dim_state;
    int dim_meas;

    Vector3d gravity_;
    Matrix3d R_world_aligned_from_imu_ = Matrix3d::Identity();
    bool yaw_alignment_initialized_ = false;

private:
    // ========================================================================
    // Kinematics
    // ========================================================================
    int num_legs_ = 4;
    int dim_legs_ = 3;

    MatrixXd p_imu_2_foot_;
    MatrixXd J_imu_2_foot_;

    VectorXd joint_position_;
    VectorXd joint_velocity_;
    VectorXd joint_effort_;

    double z_foot_ = 0;

private:
    // ========================================================================
    // Input & Measurement Buffers
    // ========================================================================
    std::vector<int> discrete_time_stack;

    std::vector<Matrix3d> R_input_rotation_stack;
    std::vector<Vector3d> accel_b_input_stack;
    std::vector<Vector3d> angular_b_input_stack;

    std::vector<MatrixXd> p_imu_2_foot_stack;
    std::vector<MatrixXd> J_imu_2_foot_stack;
    std::vector<VectorXd> contact_input_stack;
    std::vector<VectorXd> joint_velocity_stack;

private:
    // ========================================================================
    // Covariances
    // ========================================================================
    double infinite = 1e6;

    Matrix3d C_accel_ = Matrix3d::Zero();
    Matrix3d C_gyro_ = Matrix3d::Zero();

    Matrix3d C_foot_slide = Matrix3d::Zero();
    Matrix3d C_foot_swing = Matrix3d::Zero();
    Matrix3d C_bias_accel = Matrix3d::Zero();
    Matrix3d C_bias_angular = Matrix3d::Zero();

    MatrixXd C_encoder_position;
    MatrixXd C_encoder_velocity;

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    void GetMeasurement(int discrete_time);

    void InitializeIEKF();
    void UpdateIEKF();
    void CorrectKinematicsLO();
    void CorrectVelocityLO();

    void StdVec2CovMat(const std::vector<double> &std, Matrix3d &Cov);
    void StdVec2GainMat(const std::vector<double> &std, Matrix3d &Gain);

    void tic(std::string str, int mode = 0);
    void toc(std::string str);

    void quaternionToEuler(const VectorXd &quaternion, Vector3d &rpy);
    void quaternionToEuler(const Quaterniond &quaternion, Vector3d &rpy);
    void vector3dSkew(Matrix3d &skew_sym, const Vector3d &vector);
};

#endif // IEKF_HPP
