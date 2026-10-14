#pragma once
#ifndef VICON_IMU_HPP__
#define VICON_IMU_HPP__

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace base_estimators
{
  static constexpr double kG = 9.81;

  struct Imu
  {
    double t;
    Eigen::Vector3d acc; // accelerometer
    Eigen::Vector3d gyro; // gyroscope
  };

  struct Vicon
  {
    // vicon measurements are always in world frame
    double t;
    Eigen::Vector3d pos; // position
    Eigen::Quaterniond quat; // orientation
  };

  Eigen::Matrix3d skew(const Eigen::Vector3d& v);
  Eigen::Quaterniond delta_quat(const Eigen::Vector3d& dtheta);
  Eigen::Vector3d quat_so3(const Eigen::Quaterniond& q_in);

  class ImuViconESKF{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ImuViconESKF(Eigen::Vector<double, 15> cov,
                   Eigen::Vector<double, 6> sigma);

      void propagate(const Imu& imu);

      void update(const Vicon& vicon);

      Eigen::Vector3d& get_position() { return p_W_; }
      Eigen::Vector3d& get_velocity() { return v_W_; }
      Eigen::Quaterniond& get_orientation() { return q_WB_; }
      Eigen::Vector3d& get_acc_bias() { return b_a_; }
      Eigen::Vector3d& get_gyro_bias() { return b_w_; }

    protected:
      Eigen::Vector3d p_W_ = Eigen::Vector3d::Zero(); // position in world frame
      Eigen::Vector3d v_W_ = Eigen::Vector3d::Zero(); // velocity in world frame
      Eigen::Quaterniond q_WB_ = Eigen::Quaterniond::Identity(); // orientation of body frame w.r.t world frame
      Eigen::Vector3d b_a_ = Eigen::Vector3d::Zero(); // accelerometer bias
      Eigen::Vector3d b_w_ = Eigen::Vector3d::Zero(); // gyroscope bias

      Eigen::Matrix<double, 15, 15> P_ = Eigen::Matrix<double, 15, 15>::Identity(); // covariance matrix
      Eigen::Vector<double, 6> sigma_; // noise standard deviation for accelerometer and gyroscope
      // acc, gyro, ba_rw, bw_rw, p_vicon, r_vicon


      double last_t_ = std::numeric_limits<double>::quiet_NaN(); // last timestamp, start with NaN
    };

};

#endif // VICON_IMU_HPP__
