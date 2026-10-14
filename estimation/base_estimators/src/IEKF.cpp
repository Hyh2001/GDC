#include "base_estimators/IEKF.hpp"
#include <cstddef>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace
{
void validate_standard_deviations(const std::vector<double>& values, const std::string& name)
{
    for (double value : values)
        if (!std::isfinite(value) || value < 0.0 || !std::isfinite(value * value))
            throw std::invalid_argument(name + " must contain finite nonnegative standard deviations.");
}
}

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

    if (!params_ptr_ || !store_ptr_)
        throw std::invalid_argument("IEKF requires non-null parameters and sensor store.");
    if (params_ptr_->rate_ <= 0 || params_ptr_->N_ < 1 ||
        params_ptr_->N_ == std::numeric_limits<int>::max())
        throw std::invalid_argument("IEKF rate and history length must be positive.");
    if (!std::isfinite(params_ptr_->contact_theshold_) || params_ptr_->contact_theshold_ < 0.0)
        throw std::invalid_argument("Contact effort threshold must be finite and nonnegative.");
    if (params_ptr_->p_body_imu_.size() != 3 || params_ptr_->q_body_imu_.size() != 4)
        throw std::invalid_argument("IMU extrinsics require three position and four quaternion entries.");
    for (double value : params_ptr_->p_body_imu_)
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite IMU position offset.");
    const auto& imu_q = params_ptr_->q_body_imu_;
    Eigen::Quaterniond imu_orientation(imu_q[0], imu_q[1], imu_q[2], imu_q[3]);
    if (!imu_orientation.coeffs().allFinite() || !std::isfinite(imu_orientation.norm()) ||
        imu_orientation.norm() <= 1e-12)
        throw std::invalid_argument("IMU extrinsic quaternion must be finite and nonzero.");
    validate_standard_deviations({params_ptr_->orientation_init_std_, params_ptr_->foot_init_std_,
        params_ptr_->accel_bias_init_std_, params_ptr_->angular_bias_init_std_}, "Prior noise");
    Matrix3d prior_covariance;
    StdVec2CovMat(params_ptr_->p_init_std_, prior_covariance);
    StdVec2CovMat(params_ptr_->v_init_std_, prior_covariance);
    ConfigureMappings();
    dt_ = 1.0 / params_ptr_->rate_;
    N_ = params_ptr_->N_;

    dim_state = 3 + 3 + 4 + 3 + 3; // p, v, q, bias_angular, bias_accel
    dim_meas = 6 * num_contacts_;
    gravity_ << 0, 0, -9.81;

    contact_ = VectorXd::Zero(num_contacts_);
    contact_torque_ = VectorXd::Zero(num_contacts_);
    p_imu_2_foot_ = MatrixXd::Zero(3 * num_contacts_, 1);
    J_imu_2_foot_ = MatrixXd::Zero(3 * num_contacts_, num_joints_);
    joint_position_ = VectorXd::Zero(num_joints_);
    joint_velocity_ = VectorXd::Zero(num_joints_);
    // ========================================================================
    // Q: Gain/Information Matrix, inv of covariance; C: Covariance Matrix
    // ========================================================================
    x_est_ = VectorXd::Zero(dim_state);
    StdVec2CovMat(params_ptr_->accel_input_std_, C_accel_);
    StdVec2CovMat(params_ptr_->gyro_input_std_, C_gyro_);
    C_encoder_position = MakeEncoderCovariance(params_ptr_->joint_position_std_, "joint_position_std");
    C_encoder_velocity = MakeEncoderCovariance(params_ptr_->joint_velocity_std_, "joint_velocity_std");
    StdVec2CovMat(params_ptr_->foot_slide_std_, C_foot_slide);
    StdVec2CovMat(params_ptr_->foot_swing_std_, C_foot_swing);
    StdVec2CovMat(params_ptr_->accel_bias_process_std_, C_bias_accel);
    StdVec2CovMat(params_ptr_->angular_bias_process_std_, C_bias_angular);
    if (params_ptr_->using_mob_)
        mob = std::make_unique<MomentumObserver>(pin_model_, dt_);
}

void robot_IEKF::ConfigureMappings()
{
    if (pin_model_.njoints < 2 || pin_model_.joints[1].shortname() != "JointModelFreeFlyer" ||
        pin_model_.joints[1].idx_q() != 0 || pin_model_.joints[1].idx_v() != 0)
        throw std::invalid_argument("IEKF requires a free-flyer root joint.");
    if (!pin_model_.check(pin_data_))
        throw std::invalid_argument("Pinocchio data does not match the model.");
    const int root_nq = pin_model_.joints[1].nq();
    const int root_nv = pin_model_.joints[1].nv();
    for (pinocchio::JointIndex id = 2; id < static_cast<pinocchio::JointIndex>(pin_model_.njoints); ++id)
    {
        if (pin_model_.joints[id].nq() != 1 || pin_model_.joints[id].nv() != 1)
            throw std::invalid_argument("IEKF supports only scalar non-base joints: " + pin_model_.names[id]);
        auto ancestor = id;
        while (ancestor > 1) ancestor = pin_model_.parents[ancestor];
        if (ancestor != 1)
            throw std::invalid_argument("All measured joints must descend from the free flyer.");
    }
    if (params_ptr_->joint_names_.empty() ||
        params_ptr_->joint_names_.size() != static_cast<size_t>(pin_model_.nv - root_nv) ||
        pin_model_.nq - root_nq != pin_model_.nv - root_nv)
        throw std::invalid_argument("joint_names must cover every non-base movable joint.");
    num_joints_ = static_cast<int>(params_ptr_->joint_names_.size());
    std::unordered_map<pinocchio::JointIndex, int> joint_columns;
    for (int column = 0; column < num_joints_; ++column)
    {
        const auto& name = params_ptr_->joint_names_[column];
        if (!pin_model_.existJointName(name)) throw std::invalid_argument("Joint not found: " + name);
        const auto id = pin_model_.getJointId(name);
        if (id <= 1 || !joint_columns.emplace(id, column).second)
            throw std::invalid_argument("Duplicate or non-actuated joint: " + name);
        const auto& joint = pin_model_.joints[id];
        joint_mappings_.push_back({id, joint.idx_q(), joint.idx_v(),
            joint.idx_q() - root_nq, joint.idx_v() - root_nv});
    }
    if (params_ptr_->contact_names_.empty() ||
        params_ptr_->contact_names_.size() > static_cast<size_t>(std::numeric_limits<int>::max() / 6))
        throw std::invalid_argument("IEKF requires a nonempty valid contact list.");
    num_contacts_ = static_cast<int>(params_ptr_->contact_names_.size());
    if (params_ptr_->num_legs_ < 0 || params_ptr_->dim_legs_ < 0 ||
        (params_ptr_->num_legs_ != 0 && params_ptr_->num_legs_ != num_contacts_))
        throw std::invalid_argument("num_legs must match contact_names when supplied.");
    std::unordered_set<pinocchio::FrameIndex> frames;
    for (const auto& name : params_ptr_->contact_names_)
    {
        if (!pin_model_.existFrame(name)) throw std::invalid_argument("Contact frame not found: " + name);
        const auto frame = pin_model_.getFrameId(name);
        if (!frames.insert(frame).second) throw std::invalid_argument("Duplicate contact frame: " + name);
        contact_frame_ids_.push_back(frame);
        std::vector<int> columns;
        auto id = pin_model_.frames[frame].parentJoint;
        while (id > 1)
        {
            columns.push_back(joint_columns.at(id));
            id = pin_model_.parents[id];
        }
        if (id != 1) throw std::invalid_argument("Contact frame must descend from the free flyer: " + name);
        std::reverse(columns.begin(), columns.end());
        contact_joint_columns_.push_back(std::move(columns));
    }
}

MatrixXd robot_IEKF::MakeEncoderCovariance(const std::vector<double>& std, const std::string& name) const
{
    validate_standard_deviations(std, name);
    VectorXd variances(num_joints_);
    if (std.size() == static_cast<size_t>(num_joints_))
    {
        for (int i = 0; i < num_joints_; ++i) variances(i) = std[i] * std[i];
    }
    else if (std.size() == 1)
        variances.setConstant(std[0] * std[0]);
    else if (params_ptr_->dim_legs_ > 0 && std.size() == static_cast<size_t>(params_ptr_->dim_legs_))
    {
        std::vector<bool> covered(num_joints_, false);
        for (const auto& columns : contact_joint_columns_)
        {
            if (columns.size() != std.size())
                throw std::invalid_argument(name + ": legacy noise requires equally sized contact chains.");
            for (size_t i = 0; i < columns.size(); ++i)
            {
                if (covered[columns[i]])
                    throw std::invalid_argument(name + ": legacy noise is ambiguous for shared joints.");
                covered[columns[i]] = true;
                variances(columns[i]) = std[i] * std[i];
            }
        }
        if (std::find(covered.begin(), covered.end(), false) != covered.end())
            throw std::invalid_argument(name + ": legacy noise does not cover all measured joints.");
    }
    else
        throw std::invalid_argument(name + " must be scalar, per joint, or unambiguous legacy per-leg noise.");
    return variances.asDiagonal();
}

void robot_IEKF::ValidateMeasurement() const
{
    if (store_ptr_->joint_states_position_.size() != num_joints_ ||
        store_ptr_->joint_states_velocity_.size() != num_joints_ ||
        !store_ptr_->joint_states_position_.allFinite() || !store_ptr_->joint_states_velocity_.allFinite())
        throw std::invalid_argument("IEKF joint measurements must be finite and cover the model.");
    if (!store_ptr_->accel_b_.allFinite() || !store_ptr_->omega_b_.allFinite() ||
        !store_ptr_->quaternion_.coeffs().allFinite() ||
        !std::isfinite(store_ptr_->quaternion_.norm()) || store_ptr_->quaternion_.norm() <= 1e-12)
        throw std::invalid_argument("IEKF IMU measurements must be finite with a nonzero quaternion.");
    if (params_ptr_->using_mob_)
    {
        if (!mob || store_ptr_->joint_states_effort_.size() != num_joints_ ||
            !store_ptr_->joint_states_effort_.allFinite())
            throw std::invalid_argument("Momentum observation requires finite measured efforts for every joint.");
    }
    else if (store_ptr_->contact_.size() != num_contacts_ || !store_ptr_->contact_.allFinite())
        throw std::invalid_argument("Contact flags must be finite and in contact_names order.");
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
    ValidateMeasurement();
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

    accel_b_ = store_ptr_->accel_b_;
    omega_b_ = store_ptr_->omega_b_;
    p_imu_2_foot_.setZero();
    J_imu_2_foot_.setZero();

    const Vector3d offset(params_ptr_->p_body_imu_[0], params_ptr_->p_body_imu_[1],
                          params_ptr_->p_body_imu_[2]);
    VectorXd q_base0 = pinocchio::neutral(pin_model_);
    VectorXd v_base0 = VectorXd::Zero(pin_model_.nv);
    VectorXd tau_base0 = VectorXd::Zero(pin_model_.nv);
    for (int column = 0; column < num_joints_; ++column)
    {
        const auto& mapping = joint_mappings_[column];
        joint_position_(column) = store_ptr_->joint_states_position_(mapping.input_q);
        joint_velocity_(column) = store_ptr_->joint_states_velocity_(mapping.input_v);
        q_base0(mapping.idx_q) = joint_position_(column);
        v_base0(mapping.idx_v) = joint_velocity_(column);
        if (params_ptr_->using_mob_) tau_base0(mapping.idx_v) = store_ptr_->joint_states_effort_(mapping.input_v);
    }
    const int root_q = pin_model_.joints[1].idx_q();
    const int root_v = pin_model_.joints[1].idx_v();
    v_base0.segment<3>(root_v + 3) = omega_b_;
    pinocchio::forwardKinematics(pin_model_, pin_data_, q_base0);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);
    pinocchio::computeJointJacobians(pin_model_, pin_data_, q_base0);

    MatrixXd J = MatrixXd::Zero(6, pin_model_.nv);
    for (int contact = 0; contact < num_contacts_; ++contact)
    {
        const auto frame = contact_frame_ids_[contact];
        p_imu_2_foot_.block<3, 1>(3 * contact, 0) = pin_data_.oMf[frame].translation() - offset;
        J.setZero();
        pinocchio::getFrameJacobian(pin_model_, pin_data_, frame, pinocchio::LOCAL_WORLD_ALIGNED, J);
        for (int column = 0; column < num_joints_; ++column)
            J_imu_2_foot_.block<3, 1>(3 * contact, column) = J.block<3, 1>(0, joint_mappings_[column].idx_v);
    }
    if (!p_imu_2_foot_.allFinite() || !J_imu_2_foot_.allFinite())
        throw std::runtime_error("Nonfinite contact kinematics.");

    contact_torque_.setZero();
    if (params_ptr_->using_mob_)
    {
        VectorXd q_base_orien_filled = q_base0;
        q_base_orien_filled.segment<4>(root_q + 3) = quat_gt_.coeffs(); // xyzw
        if (!isinit) mob->init(q_base_orien_filled, v_base0);
        mob->update(q_base_orien_filled, v_base0, tau_base0);
        const VectorXd forces = EstimateContactForces(mob->getResidual());
        for (int contact = 0; contact < num_contacts_; ++contact)
            contact_torque_(contact) = forces(3 * contact + 2) > params_ptr_->contact_theshold_ ? 1.0 : 0.0;
        contact_ = contact_torque_;
    }
    else
        contact_ = store_ptr_->contact_;
    isinit = true;
    if (params_ptr_->visualize_)
    {
        std::cout << "[robot_IEKF] contact: " << contact_.transpose() << std::endl;
        std::cout << "[robot_IEKF] contact torque: " << contact_torque_.transpose() << std::endl;
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
    if (std.size() != 3) throw std::invalid_argument("Three noise standard deviations are required.");
    validate_standard_deviations(std, "Noise");
    Cov.setZero();
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

VectorXd robot_IEKF::EstimateContactForces(const VectorXd& residual) const
{
    if (residual.size() != pin_model_.nv || !residual.allFinite())
        throw std::invalid_argument("Momentum residual must be finite and in model velocity order.");
    VectorXd measured_residual(num_joints_);
    for (int column = 0; column < num_joints_; ++column)
        measured_residual(column) = residual(joint_mappings_[column].idx_v);
    // A = [J_1^T ... J_m^T]. Shared joint rows couple candidate contact forces.
    const MatrixXd A = J_imu_2_foot_.transpose();
    MatrixXd normal = A.transpose() * A;
    normal.diagonal().array() += 1e-4; // Existing lambda=0.01, squared.
    const VectorXd forces = normal.ldlt().solve(A.transpose() * measured_residual);
    if (!forces.allFinite()) throw std::runtime_error("Nonfinite contact force estimate.");
    return forces;
}

Matrix3d robot_IEKF::GetContactPositionCovariance(int contact) const
{
    const auto J = J_imu_2_foot_stack.back().block(3 * contact, 0, 3, num_joints_);
    return J * C_encoder_position * J.transpose();
}

Vector3d robot_IEKF::GetContactVelocity(int contact) const
{
    const auto J = J_imu_2_foot_stack.back().block(3 * contact, 0, 3, num_joints_);
    const Vector3d p = p_imu_2_foot_stack.back().block<3, 1>(3 * contact, 0);
    return -J * joint_velocity_stack.back() - angular_b_input_stack.back().cross(p);
}

Matrix3d robot_IEKF::GetContactVelocityCovariance(int contact) const
{
    const auto J = J_imu_2_foot_stack.back().block(3 * contact, 0, 3, num_joints_);
    Matrix3d omega_skew, position_skew;
    vector3dSkew(omega_skew, angular_b_input_stack.back());
    vector3dSkew(position_skew, p_imu_2_foot_stack.back().block<3, 1>(3 * contact, 0));
    const MatrixXd omega_J = omega_skew * J;
    // Retains the existing approximation, excluding dJ/dq * joint_velocity.
    return J * C_encoder_velocity * J.transpose() +
        omega_J * C_encoder_position * omega_J.transpose() +
        position_skew * C_gyro_ * position_skew.transpose();
}

void robot_IEKF::CorrectKinematicsLO()
{
    std::vector<std::pair<int, bool>> contacts;
    for (int i = 0; i < num_contacts_; ++i)
        contacts.emplace_back(i, bool(contact_input_stack.back()(i)));
    filter.setContacts(contacts);

    vectorKinematics measured_kinematics;
    for (int i = 0; i < num_contacts_; ++i)
    {
        Eigen::Matrix4d pose_i = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 6, 6> covariance_i = Eigen::Matrix<double, 6, 6>::Zero();
        pose_i.block<3, 1>(0, 3) = p_imu_2_foot_stack.back().block<3, 1>(i * 3, 0);
        covariance_i.block<3, 3>(3, 3) = GetContactPositionCovariance(i);
        measured_kinematics.emplace_back(i, pose_i, covariance_i);
    }
    filter.CorrectKinematics(measured_kinematics);
}

void robot_IEKF::CorrectVelocityLO()
{
    vectorTwistBody measured_twist_body;
    for (int i = 0; i < num_contacts_; ++i)
    {
        if (contact_input_stack.back()(i) != 1.0) continue;
        Eigen::Matrix<double, 6, 1> twist_body_i = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 6> covariance_i = Eigen::Matrix<double, 6, 6>::Zero();
        twist_body_i.tail<3>() = GetContactVelocity(i);
        covariance_i.block<3, 3>(3, 3) = GetContactVelocityCovariance(i);
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
    ValidateMeasurement();
    if (accel_b_input_stack.empty())
        throw std::logic_error("Initialize the IEKF before updating it.");

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
    for (size_t i = 0; i < num_contacts_; i++)
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
