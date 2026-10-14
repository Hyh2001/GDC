#include "base_estimators/vicon_imu.hpp"

namespace base_estimators
{
  ImuViconESKF::ImuViconESKF(Eigen::Vector<double, 15> cov,
                             Eigen::Vector<double, 6> sigma)
  {
    P_ = cov.asDiagonal();
    sigma_ = sigma;
    last_t_ = std::numeric_limits<double>::quiet_NaN();
  }

  void ImuViconESKF::propagate(const Imu& imu)
  {
    if (std::isnan(last_t_))
    {
      last_t_ = imu.t;
      return;
    }

    double dt = imu.t - last_t_;
    if (dt <= 0.0)
    {
      std::cerr << "Non-positive time difference between IMU measurements: " << dt << "s" << std::endl;
      return;
    }
    last_t_ = imu.t;

    Eigen::Vector3d g_W(0.0, 0.0, -kG);
    Eigen::Vector3d omega = imu.gyro - b_w_;
    Eigen::Vector3d acc = imu.acc - b_a_;

    // state propagation
    q_WB_ = (q_WB_ * delta_quat(omega * dt)).normalized();
    Eigen::Matrix3d R_WB = q_WB_.toRotationMatrix();
    Eigen::Vector3d a_W = R_WB * acc + g_W;
    p_W_ = p_W_ + v_W_ * dt + 0.5 * a_W * dt * dt;
    v_W_ = v_W_ + a_W * dt;

    // covariance propagation
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
    Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();

    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
    F.block<3,3>(3,6) = -R_WB * skew(acc);
    F.block<3,3>(3,9) = -R_WB;
    F.block<3,3>(6,6) = -skew(omega);
    F.block<3,3>(6,12)= -Eigen::Matrix3d::Identity();

    G.block<3,3>(3,0)  = R_WB;
    G.block<3,3>(6,3)  = Eigen::Matrix3d::Identity();
    G.block<3,3>(9,6)  = Eigen::Matrix3d::Identity();
    G.block<3,3>(12,9) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
    Qc.block<3,3>(0,0) = sigma_[0] * sigma_[0] * Eigen::Matrix3d::Identity();
    Qc.block<3,3>(3,3) = sigma_[1] * sigma_[1] * Eigen::Matrix3d::Identity();
    Qc.block<3,3>(6,6) = sigma_[2] * sigma_[2] * Eigen::Matrix3d::Identity();
    Qc.block<3,3>(9,9) = sigma_[3] * sigma_[3] * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 15> Phi =
        Eigen::Matrix<double, 15, 15>::Identity() + F * dt;

    Eigen::Matrix<double, 15, 15> Qd =
        (G * Qc * G.transpose()) * dt;

    P_ = Phi * P_ * Phi.transpose() + Qd;
    P_ = 0.5 * (P_ + P_.transpose());
  }

  void ImuViconESKF::update(const Vicon& vicon)
  {
    Eigen::Vector3d r_p = p_W_ - vicon.pos;
    Eigen::Quaterniond q_err =
        vicon.quat.conjugate() * q_WB_;

    Eigen::Vector3d r_theta = quat_so3(q_err);
    Eigen::Matrix<double, 6, 1> r;
    r << r_p, r_theta;

    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(3,6) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    R.block<3,3>(0,0) = sigma_[4] * sigma_[4] * Eigen::Matrix3d::Identity();
    R.block<3,3>(3,3) = sigma_[5] * sigma_[5] * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 15, 6> K = P_ * H.transpose() * S.inverse();
    Eigen::Matrix<double, 15, 1> dx = -K * r;
    Eigen::Matrix<double, 15, 15> I =
        Eigen::Matrix<double, 15, 15>::Identity();
    P_ = (I - K*H)*P_*(I - K*H).transpose() + K*R*K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    // Inject
    p_W_ += dx.block<3,1>(0,0);
    v_W_ += dx.block<3,1>(3,0);
    q_WB_ = (q_WB_ * delta_quat(dx.block<3,1>(6,0))).normalized();
    b_a_ += dx.block<3,1>(9,0);
    b_w_ += dx.block<3,1>(12,0);
  }


  /*
    Helper functions
  */
  Eigen::Matrix3d skew(const Eigen::Vector3d& v)
  {
    Eigen::Matrix3d S;
    S <<  0.0, -v.z(), v.y(),
          v.z(), 0.0, -v.x(),
          -v.y(), v.x(), 0.0;
    return S;
  }

  Eigen::Quaterniond delta_quat(const Eigen::Vector3d& dtheta)
  {
    double angle = dtheta.norm();

    if (angle < 1e-12)
    {
        return Eigen::Quaterniond(1.0,
                                  0.5 * dtheta.x(),
                                  0.5 * dtheta.y(),
                                  0.5 * dtheta.z()).normalized();
    }

    Eigen::Vector3d axis = dtheta / angle;
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  }

  Eigen::Vector3d quat_so3(const Eigen::Quaterniond& q_in)
  {
    Eigen::Quaterniond q = q_in;

    if (q.w() < 0.0)
        q.coeffs() *= -1.0;

    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
  }



};
