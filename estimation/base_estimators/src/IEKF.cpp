#include "base_estimators/IEKF.hpp"
#include <cstddef>
#include <fstream>

robot_IEKF::robot_IEKF(pinocchio::Model &pin_model,
                       pinocchio::Data &pin_data,
                       std::shared_ptr<robot_store> store,
                       std::shared_ptr<robot_params> params)
    : pin_model_(pin_model),
      pin_data_(pin_data),
      store_ptr_(store),
      params_ptr_(params)
{

    R_world_aligned_from_imu_.setIdentity();
    yaw_alignment_initialized_ = true;

    dt_ = 1.0 / params_ptr_->rate_;
    N_ = params_ptr_->N_;
    num_legs_ = params_ptr_->num_legs_;
    dim_legs_ = params_ptr_->dim_legs_;

    dim_state = 3 + 3 + 4 + 3 + 3; // p, v, q, bias_angular, bias_accel
    dim_meas = 6 * num_legs_;
    gravity_ << 0, 0, -9.81;

    mob = new MomentumObserver(pin_model_, dt_);
    contact_ = VectorXd::Zero(num_legs_);
    contact_torque_ = VectorXd::Zero(num_legs_);
    // ========================================================================
    // Q: Gain/Information Matrix, inv of covariance; C: Covariance Matrix
    // ========================================================================
    x_est_ = VectorXd::Zero(dim_state);
    StdVec2CovMat(params_ptr_->accel_input_std_, C_accel_);
    StdVec2CovMat(params_ptr_->gyro_input_std_, C_gyro_);
    StdVec2CovMat(params_ptr_->joint_position_std_, C_encoder_position);
    StdVec2CovMat(params_ptr_->joint_velocity_std_, C_encoder_velocity);
    StdVec2CovMat(params_ptr_->foot_slide_std_, C_foot_slide);
    StdVec2CovMat(params_ptr_->foot_swing_std_, C_foot_swing);
    StdVec2CovMat(params_ptr_->accel_bias_process_std_, C_bias_accel);
    StdVec2CovMat(params_ptr_->angular_bias_process_std_, C_bias_angular);
}

void robot_IEKF::initialize()
{
    InitializeIEKF();
    UpdateIEKF();
}

void robot_IEKF::update(int T)
{
    UpdateIEKF();
}

// Tested
void robot_IEKF::GetMeasurement(int T)
{
    // Align the yaw of init with the start config as zero yaw
    quat_gt_ = store_ptr_->quaternion_.normalized();
    Matrix3d R_gt_raw = quat_gt_.toRotationMatrix();
    if (!yaw_alignment_initialized_)
    {
        // Extract yaw from rotation matrix (ZYX convention)
        double yaw0 = std::atan2(R_gt_raw(1, 0), R_gt_raw(0, 0));

        // Construct Rz(-yaw0)
        double c = std::cos(-yaw0);
        double s = std::sin(-yaw0);

        R_world_aligned_from_imu_ << c, -s, 0,
            s, c, 0,
            0, 0, 1;
        yaw_alignment_initialized_ = true;

        std::cout << "[robot_IEKF] Initialize world yaw alignment, yaw0 = "
                  << yaw0 << std::endl;
    }
    R_gt_ = R_world_aligned_from_imu_ * R_gt_raw;
    quat_gt_ = Quaterniond(R_gt_);
    quat_gt_.normalize();
    q_gt << quat_gt_.w(), quat_gt_.x(), quat_gt_.y(), quat_gt_.z();
    quaternionToEuler(q_gt, rpy_gt_);

    accel_b_ = store_ptr_->accel_b_; // returns the gravity free acceleration in body frame;
    omega_b_ = store_ptr_->omega_b_;
    joint_position_ = store_ptr_->joint_states_position_;
    joint_velocity_ = store_ptr_->joint_states_velocity_;

    p_imu_2_foot_ = MatrixXd::Zero(3 * num_legs_, 1);
    J_imu_2_foot_ = MatrixXd::Zero(3 * num_legs_, dim_legs_);

    MatrixXd off_set = VectorXd::Zero(3);
    off_set << params_ptr_->p_body_imu_[0],
        params_ptr_->p_body_imu_[1],
        params_ptr_->p_body_imu_[2]; // unitree base to unitree imu

    Eigen::VectorXd q_base0 = Eigen::VectorXd::Zero(pin_model_.nq);
    q_base0(3) = 0.0; // x
    q_base0(4) = 0.0; // y
    q_base0(5) = 0.0; // z
    q_base0(6) = 1.0; // w
    q_base0.segment(7, pin_model_.nq - 7) = joint_position_;

    // Forward kinematics
    Eigen::VectorXd q_base_orien_filled = q_base0;
    q_base_orien_filled(3) = q_gt(1); // x
    q_base_orien_filled(4) = q_gt(2); // y
    q_base_orien_filled(5) = q_gt(3); // z
    q_base_orien_filled(6) = q_gt(0); // w
    pinocchio::forwardKinematics(pin_model_, pin_data_, q_base0);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);
    pinocchio::computeJointJacobians(pin_model_, pin_data_, q_base0);

    Eigen::VectorXd v_base0 = Eigen::VectorXd::Zero(pin_model_.nv);
    v_base0.segment(6, pin_model_.nv - 6) = joint_velocity_;
    v_base0.segment(3, 3) = omega_b_;
    Eigen::VectorXd tau_base0 = Eigen::VectorXd::Zero(pin_model_.nv);
    tau_base0.segment(6, pin_model_.nv - 6) = store_ptr_->joint_states_effort_;

    if (!isinit)
    {
        mob->init(q_base_orien_filled, v_base0);
        isinit = true;
    }
    mob->update(q_base_orien_filled, v_base0, tau_base0);

    Eigen::VectorXd residual = mob->getResidual();
    // std::cout << mob->getResidual().transpose() << std::endl;
    // Loop over legs
    for (int i = 0; i < num_legs_; ++i)
    {
        pinocchio::FrameIndex foot_id =
            pin_model_.getFrameId(params_ptr_->contact_names_[i]);

        // --------------------------------------------------
        // Foot position in imu frame
        // --------------------------------------------------
        Eigen::Vector3d p_foot = pin_data_.oMf[foot_id].translation();
        p_foot -= off_set; // offset from body to imu
        p_imu_2_foot_.block<3, 1>(i * 3, 0) = p_foot;

        // --------------------------------------------------
        // Foot Jacobian (imu frame)
        // --------------------------------------------------
        Eigen::MatrixXd J(6, pin_model_.nv);
        J.setZero();
        pinocchio::getFrameJacobian(pin_model_, pin_data_, foot_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
        Eigen::MatrixXd J_linear = J.topRows<3>();
        J_imu_2_foot_.block(i * 3, 0, 3, dim_legs_) = J_linear.block(0, 6 + i * dim_legs_, 3, dim_legs_);

        // --------------------------------------------------
        // Contact Flagging trick
        // --------------------------------------------------
        Eigen::VectorXd r_leg =
            residual.segment(6 + i * dim_legs_, dim_legs_);

        // J is 3 x nv
        Eigen::MatrixXd JT = J_linear.transpose(); // nv x 3

        // Compute A = J J^T + λ² I   (3 x 3)
        double lambda = 1e-2; // try 1e-2, increase if still unstable

        Eigen::MatrixXd A =
            J_linear * JT +
            lambda * lambda *
                Eigen::MatrixXd::Identity(J_linear.rows(), J_linear.rows());

        // Solve (J J^T + λ²I) y = r
        Eigen::VectorXd y =
            A.ldlt().solve(r_leg);

        // Final force estimate
        Eigen::VectorXd f =
            JT * y;

        contact_torque_(i) = (-f(2) > params_ptr_->contact_theshold_) ? 1.0 : 0.0;
    }
    // std::cout << "[robot_IEKF] residual: " << residual.segment(6, 12).transpose() << std::endl;
    contact_ = store_ptr_->contact_;
    if (params_ptr_->using_mob_)
    {
        contact_ = contact_torque_;
    }
    if (params_ptr_->visualize_)
    {
        std::cout << "--------------------" << std::endl;
        std::cout << "[robot_IEKF] contact: " << contact_.transpose() << std::endl;
        std::cout << "[robot_IEKF] contact_torque: " << contact_torque_.transpose() << std::endl;
    }

    discrete_time_stack.push_back(T);
    R_input_rotation_stack.push_back(R_gt_);
    accel_b_input_stack.push_back(accel_b_);
    p_imu_2_foot_stack.push_back(p_imu_2_foot_);
    J_imu_2_foot_stack.push_back(J_imu_2_foot_);
    contact_input_stack.push_back(contact_);
    joint_velocity_stack.push_back(joint_velocity_);
    angular_b_input_stack.push_back(omega_b_);

    if (int(accel_b_input_stack.size()) > N_ + 1)
    {
        discrete_time_stack.erase(discrete_time_stack.begin());
        R_input_rotation_stack.erase(R_input_rotation_stack.begin());
        accel_b_input_stack.erase(accel_b_input_stack.begin());
        p_imu_2_foot_stack.erase(p_imu_2_foot_stack.begin());
        J_imu_2_foot_stack.erase(J_imu_2_foot_stack.begin());
        contact_input_stack.erase(contact_input_stack.begin());
        joint_velocity_stack.erase(joint_velocity_stack.begin());
        angular_b_input_stack.erase(angular_b_input_stack.begin());
    }
}

void robot_IEKF::StdVec2CovMat(const std::vector<double> &std, Matrix3d &Cov)
{
    Cov.diagonal() << std::pow(std[0], 2),
        std::pow(std[1], 2),
        std::pow(std[2], 2);
}

void robot_IEKF::StdVec2GainMat(const std::vector<double> &std, Matrix3d &Gain)
{
    Gain.diagonal() << 1 / std::pow(std[0], 2),
        1 / std::pow(std[1], 2),
        1 / std::pow(std[2], 2);
}

void robot_IEKF::tic(std::string str, int mode)
{
    static std::chrono::_V2::system_clock::time_point t_start;

    if (mode == 0)
        t_start = std::chrono::high_resolution_clock::now();
    else
    {
        auto t_end = std::chrono::high_resolution_clock::now();
        std::cout << str + " elapsed time: " << (t_end - t_start).count() * 1E-9 << " seconds\n";
    }
}

void robot_IEKF::toc(std::string str) { tic(str, 1); }

void robot_IEKF::quaternionToEuler(const VectorXd &quaternion, Vector3d &rpy)
{
    double roll, pitch, yaw;

    double qw = quaternion(0);
    double qx = quaternion(1);
    double qy = quaternion(2);
    double qz = quaternion(3);
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // Use M_PI for pi in C++
    else
        pitch = asin(sinp);
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
    rpy << roll, pitch, yaw;
}

void robot_IEKF::quaternionToEuler(const Quaterniond &quaternion, Vector3d &rpy)
{
    double roll, pitch, yaw;

    double qw = quaternion.w();
    double qx = quaternion.x();
    double qy = quaternion.y();
    double qz = quaternion.z();
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // Use M_PI for pi in C++
    else
        pitch = asin(sinp);
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
    rpy << roll, pitch, yaw;
}

void robot_IEKF::vector3dSkew(Matrix3d &skew_sym, const Vector3d &vector)
{

    skew_sym << 0, -vector(2), vector(1),
        vector(2), 0, -vector(0),
        -vector(1), vector(0), 0;
}

void robot_IEKF::CorrectKinematicsLO()
{
    std::vector<std::pair<int, bool>> contacts;
    for (int i = 0; i < num_legs_; i++)
        contacts.emplace_back(i, bool(contact_input_stack.back()(i)));
    filter.setContacts(contacts);

    vectorKinematics measured_kinematics;
    for (int i = 0; i < num_legs_; i++)
    {
        Eigen::Matrix4d pose_i = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 6, 6> covariance_i = MatrixXd::Zero(6, 6);
        pose_i.block<3, 1>(0, 3) = p_imu_2_foot_stack.back().block<3, 1>(i * 3, 0);

        MatrixXd C_meas_input = MatrixXd::Zero(2 * dim_legs_ + 3, 2 * dim_legs_ + 3);
        C_meas_input.block(0, 0, dim_legs_, dim_legs_) = C_encoder_velocity;
        C_meas_input.block(dim_legs_, dim_legs_, dim_legs_, dim_legs_) = C_encoder_position;
        C_meas_input.block<3, 3>(2 * dim_legs_, 2 * dim_legs_) = C_gyro_;

        MatrixXd G_meas_pos_i = MatrixXd::Zero(3, 2 * dim_legs_ + 3);
        G_meas_pos_i.block(0, 3, 3, dim_legs_) = J_imu_2_foot_stack.back().block(i * 3, 0, 3, dim_legs_);

        covariance_i.block<3, 3>(3, 3) = G_meas_pos_i * C_meas_input * G_meas_pos_i.transpose();
        // covariance_i.block<3, 3>(3, 3) = C_meas_input.block(0, 0, dim_legs_, dim_legs_);

        measured_kinematics.emplace_back(i, pose_i, covariance_i);
    }
    filter.CorrectKinematics(measured_kinematics);
}

void robot_IEKF::CorrectVelocityLO()
{
    vectorTwistBody measured_twist_body;
    for (int i = 0; i < num_legs_; i++)
    {
        if (contact_input_stack.back()(i) != 1.0)
            continue;

        Eigen::Matrix<double, 6, 1> twist_body_i;
        Eigen::Matrix<double, 6, 6> covariance_i = MatrixXd::Zero(6, 6);
        twist_body_i.block<3, 1>(0, 0) << 0, 0, 0;
        twist_body_i.block<3, 1>(3, 0) =
            -J_imu_2_foot_stack.back().block(i * 3, 0, 3, dim_legs_) *
                joint_velocity_stack.back().segment(i * dim_legs_, dim_legs_) -
            angular_b_input_stack.back().cross(
                p_imu_2_foot_stack.back().block<3, 1>(i * 3, 0));

        MatrixXd C_meas_input = MatrixXd::Zero(2 * dim_legs_ + 3, 2 * dim_legs_ + 3);
        C_meas_input.block(0, 0, dim_legs_, dim_legs_) = C_encoder_velocity;
        C_meas_input.block(dim_legs_, dim_legs_, dim_legs_, dim_legs_) = C_encoder_position;
        C_meas_input.block<3, 3>(2 * dim_legs_, 2 * dim_legs_) = C_gyro_;

        MatrixXd G_meas_vel_i = MatrixXd::Zero(3, 2 * dim_legs_ + 3);
        G_meas_vel_i.block(0, 0, 3, dim_legs_) = -J_imu_2_foot_stack.back().block(i * 3, 0, 3, dim_legs_);

        Matrix3d omega_skew;
        vector3dSkew(omega_skew, angular_b_input_stack.back());
        G_meas_vel_i.block(0, 3, 3, dim_legs_) =
            -omega_skew * J_imu_2_foot_stack.back().block(i * 3, 0, 3, dim_legs_);

        Matrix3d kin_skew;
        vector3dSkew(kin_skew,
                     p_imu_2_foot_stack.back().block<3, 1>(i * 3, 0));
        G_meas_vel_i.block<3, 3>(0, 6) = kin_skew;

        covariance_i.block<3, 3>(3, 3) =
            G_meas_vel_i * C_meas_input * G_meas_vel_i.transpose();
        // covariance_i.block<3, 3>(3, 3) = C_meas_input.block(0, 0, dim_legs_, dim_legs_);

        measured_twist_body.emplace_back(i, twist_body_i, covariance_i);
    }
    filter.CorrectVelocityBody(measured_twist_body);
}

// KF part validated
void robot_IEKF::InitializeIEKF()
{
    GetMeasurement(0);

    // Initialization of robot states in RobotState
    RobotState initial_state;
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;

    R0 = R_gt_; // initial orientation

    v0 << 0, 0, 0;  // initial velocity
    p0 << 0, 0, 0;  // initial position
    bg0 << 0, 0, 0; // initial gyroscope bias
    ba0 << 0, 0, 0; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialization of Prior distribution in RobotState
    MatrixXd P_init = MatrixXd::Zero(15, 15); // p, v, orientation, bias_angular, bias_accel

    Matrix3d C_pint = Matrix3d::Zero();
    Matrix3d C_vint = Matrix3d::Zero();

    StdVec2CovMat(params_ptr_->p_init_std_, C_pint);
    StdVec2CovMat(params_ptr_->v_init_std_, C_vint);

    P_init.block<3, 3>(0, 0) = C_pint;
    P_init.block<3, 3>(3, 3) = C_vint;
    P_init.block<3, 3>(6, 6) = std::pow(params_ptr_->orientation_init_std_, 2) * Matrix3d::Identity();
    P_init.block<3, 3>(9, 9) = std::pow(params_ptr_->angular_bias_init_std_, 2) * Matrix3d::Identity();
    P_init.block<3, 3>(12, 12) = std::pow(params_ptr_->accel_bias_init_std_, 2) * Matrix3d::Identity();

    initial_state.setP(P_init);

    // Initialize state covariance
    NoiseParams noise_params;
    noise_params.setGyroscopeNoise(C_gyro_);
    noise_params.setAccelerometerNoise(C_accel_);
    noise_params.setGyroscopeBiasNoise(C_bias_angular);
    noise_params.setAccelerometerBiasNoise(C_bias_accel);
    noise_params.setContactNoise(C_foot_slide);

    // Initialize filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);

    if (params_ptr_->visualize_)
    {
        std::cout << "Initial state is: \n";
        std::cout << filter.getState() << std::endl;
        std::cout << "Noise parameters are initialized to: \n";
        std::cout << filter.getNoiseParams() << std::endl;
        std::cout << "Robot's state is initialized to: \n";
        std::cout << filter.getState() << std::endl;
    }

    // Correction update
    //---------------------------------------------------------------
    if (params_ptr_->using_lo_p_)
        CorrectKinematicsLO();
    if (params_ptr_->using_lo_v_)
        CorrectVelocityLO();

    RobotState X_est = filter.getState();
    x_est_.segment(0, 3) = X_est.getPosition();
    x_est_.segment(3, 3) = X_est.getVelocity();
    Matrix3d R_sb = X_est.getRotation();
    Quaterniond q_sb(R_sb);
    x_est_.segment(6, 4) << q_sb.w(), q_sb.x(), q_sb.y(), q_sb.z();
    x_est_.segment(10, 3) = X_est.getGyroscopeBias();
    x_est_.segment(13, 3) = X_est.getAccelerometerBias();
    quaternionToEuler(q_sb, rpy_est_);
}

void robot_IEKF::UpdateIEKF()
{

    // Prediction update
    //---------------------------------------------------------------
    Vector3d accel_b = accel_b_input_stack.back();
    Vector3d angular_b = angular_b_input_stack.back();

    Matrix<double, 6, 1> imu_measurement = Matrix<double, 6, 1>::Zero();
    imu_measurement.segment(0, 3) = angular_b;
    imu_measurement.segment(3, 3) = accel_b;
    filter.Propagate(imu_measurement, dt_);

    // Correction update
    //---------------------------------------------------------------
    GetMeasurement(0);
    if (params_ptr_->using_lo_p_)
        CorrectKinematicsLO();
    if (params_ptr_->using_lo_v_)
        CorrectVelocityLO();

    RobotState X_est = filter.getState();
    x_est_.segment(0, 3) = X_est.getPosition();
    x_est_.segment(3, 3) = X_est.getVelocity();
    Matrix3d R_sb = X_est.getRotation();
    Quaterniond q_sb(R_sb);
    x_est_.segment(6, 4) << q_sb.w(), q_sb.x(), q_sb.y(), q_sb.z();
    x_est_.segment(10, 3) = X_est.getGyroscopeBias();
    x_est_.segment(13, 3) = X_est.getAccelerometerBias();

    quaternionToEuler(q_sb, rpy_est_);

    // ========================================================================
    // Post Processing of the estimated state
    // ========================================================================

    // Plane contact height drift compensation
    //---------------------------------------------------------------
    // z_foot_ is the average height of the contact feet in the world frame, which should be zero on the ground.
    // z_est - z_foot can be used to prevent drift of height
    // if robot is flying, still minus z_foot_ at the beginning of flying phase to make sure consistence
    size_t n_contact = 0;
    double z_foot_tmp = 0;
    for (size_t i = 0; i < num_legs_; i++)
    {
        if (contact_[i])
        {
            n_contact++;
            Vector3d p_imu2foot_w = X_est.getPosition() + R_sb * p_imu_2_foot_.block<3, 1>(3 * i, 0);
            z_foot_tmp += p_imu2foot_w(2);
        }
    }
    if (n_contact > 0)
    {
        z_foot_ = z_foot_tmp / n_contact;
    }
    x_est_[2] -= z_foot_;
    x_est_[2] += 0.05; // bias adjustments

    // Compute body velocity in base frame but the imu frame
    //---------------------------------------------------------------
    Vector3d p_body_imu(params_ptr_->p_body_imu_[0],
                        params_ptr_->p_body_imu_[1],
                        params_ptr_->p_body_imu_[2]);

    // IMU velocity in world frame
    Vector3d v_imu_w = x_est_.segment(3, 3);
    // Convert to base origin velocity (world frame)
    Vector3d v_base_w = v_imu_w - R_sb * (angular_b_input_stack.back().cross(p_body_imu));
    v_b_est_ = R_sb.transpose() * v_base_w;

    if (params_ptr_->visualize_)
    {
        std::cout << "Estimated state is: \n";
        std::cout << x_est_.transpose() << std::endl;
    }
}
