#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <numeric>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "base_estimators/IEKF.hpp"

// Exercise production measurement helpers and filter state without widening the public API.
struct IEKFTestAccess
{
  static void measure(robot_IEKF & estimator)
  {
    estimator.GetMeasurement(0);
  }
  static const MatrixXd & positions(const robot_IEKF & estimator)
  {
    return estimator.p_imu_2_foot_;
  }
  static const MatrixXd & jacobian(const robot_IEKF & estimator)
  {
    return estimator.J_imu_2_foot_;
  }
  static const MatrixXd & position_noise(const robot_IEKF & estimator)
  {
    return estimator.C_encoder_position;
  }
  static const MatrixXd & velocity_noise(const robot_IEKF & estimator)
  {
    return estimator.C_encoder_velocity;
  }
  static Matrix3d position_covariance(const robot_IEKF & estimator, int contact)
  {
    return estimator.GetContactPositionCovariance(contact);
  }
  static Matrix3d velocity_covariance(const robot_IEKF & estimator, int contact)
  {
    return estimator.GetContactVelocityCovariance(contact);
  }
  static Vector3d velocity(const robot_IEKF & estimator, int contact)
  {
    return estimator.GetContactVelocity(contact);
  }
  static VectorXd forces(const robot_IEKF & estimator, const VectorXd & residual)
  {
    return estimator.EstimateContactForces(residual);
  }
  static const auto & support(const robot_IEKF & estimator)
  {
    return estimator.contact_joint_columns_;
  }
  static bool has_observer(const robot_IEKF & estimator)
  {
    return bool(estimator.mob);
  }
  static size_t history_size(const robot_IEKF & estimator)
  {
    return estimator.joint_velocity_stack.size();
  }
  static MatrixXd filter_covariance(robot_IEKF & estimator)
  {
    return estimator.filter.getState().getP();
  }
};

namespace
{
struct Robot
{
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  std::shared_ptr<robot_params> params = std::make_shared<robot_params>();
  std::shared_ptr<robot_store> store = std::make_shared<robot_store>();

  explicit Robot(
    std::vector<int> leg_sizes = {4, 4}, bool arm = false, bool shared = false,
    bool reverse_model_branches = false)
  {
    const auto root = add_joint(0, pinocchio::JointModelFreeFlyer(), Vector3d::Zero(), "root");
    if (arm) {
      const auto shoulder = add_joint(
        root, pinocchio::JointModelRY(), Vector3d(.1, .3, .2),
        "arm0");
      add_joint(shoulder, pinocchio::JointModelRX(), Vector3d(.3, 0, 0), "arm1");
    }
    auto branch_root = root;
    if (shared) {
      branch_root = add_joint(root, pinocchio::JointModelRZ(), Vector3d(0, 0, -.1), "waist");
    }
    std::vector<pinocchio::JointIndex> parents(leg_sizes.size(), branch_root);
    auto add_leg_joint = [&](size_t leg, int depth)
    {
      const Vector3d translation(depth == 0 ? .1 : .06, depth == 0 ? .25 * (1.0 - 2.0 * leg) : .02,
        -.2);
      pinocchio::JointModel joint;
      if (depth % 3 == 0) {
        joint = pinocchio::JointModelRX();
      } else if (depth % 3 == 1) {
        joint = pinocchio::JointModelRY();
      } else {
        joint = pinocchio::JointModelRZ();
      }
      parents[leg] =
        add_joint(
        parents[leg], joint, translation,
        "leg" + std::to_string(leg) + "_" + std::to_string(depth));
    };
    // Pinocchio requires depth-first model order; reverse whole branches while
    // allowing the external joint_names configuration to be fully interleaved.
    for (size_t order = 0; order < leg_sizes.size(); ++order) {
      const size_t leg = reverse_model_branches ? leg_sizes.size() - order - 1 : order;
      for (int depth = 0; depth < leg_sizes[leg]; ++depth) {add_leg_joint(leg, depth);}
    }
    for (size_t leg = 0; leg < leg_sizes.size(); ++leg) {
      const auto name = "sole" + std::to_string(leg);
      // Represents a rigid contact frame below the last moving joint.
      model.addFrame(
        pinocchio::Frame(
          name, parents[leg], pinocchio::SE3(Matrix3d::Identity(), Vector3d(.13, -.04, -.21)),
          pinocchio::OP_FRAME),
        false);
      params->contact_names_.push_back(name);
    }
    for (pinocchio::JointIndex id = 2; id < static_cast<pinocchio::JointIndex>(model.njoints);
      ++id)
    {
      params->joint_names_.push_back(model.names[id]);
    }
    data = std::make_unique<pinocchio::Data>(model);
    params->rate_ = 500;
    params->N_ = 2;
    params->using_lo_p_ = 1;
    params->using_lo_v_ = 1;
    params->using_mob_ = 0;
    params->visualize_ = 0;
    params->contact_theshold_ = 12;
    params->p_body_imu_ = {.01, -.02, .03};
    params->q_body_imu_ = {1, 0, 0, 0};
    params->p_init_std_ = {.001, .002, .003};
    params->v_init_std_ = {.001, .002, .003};
    params->orientation_init_std_ = .001;
    params->foot_init_std_ = .001;
    params->accel_bias_init_std_ = params->angular_bias_init_std_ = .001;
    params->accel_input_std_ = {.03, .04, .05};
    params->gyro_input_std_ = {.02, .03, .04};
    params->accel_bias_process_std_ = params->angular_bias_process_std_ = {.001, .001, .001};
    params->foot_slide_std_ = {.01, .02, .03};
    params->foot_swing_std_ = {1000, 1000, 1000};
    set_noise();
    store->joint_states_position_ = VectorXd::Zero(model.nq - 7);
    store->joint_states_velocity_ = VectorXd::Zero(model.nv - 6);
    store->joint_states_effort_ = VectorXd::Zero(model.nv - 6);
    for (pinocchio::JointIndex id = 2; id < static_cast<pinocchio::JointIndex>(model.njoints);
      ++id)
    {
      const auto & name = model.names[id];
      store->joint_states_position_(model.joints[id].idx_q() - 7) = value_for_name(name);
      store->joint_states_velocity_(model.joints[id].idx_v() - 6) = value_for_name(name) * .1;
    }
    store->contact_ = VectorXd::Ones(leg_sizes.size());
    store->quaternion_ = Quaterniond::Identity();
    store->accel_b_ = Vector3d(.01, -.02, 9.81);
    store->omega_b_ = Vector3d(.04, -.06, .08);
  }

  pinocchio::JointIndex add_joint(
    pinocchio::JointIndex parent, const pinocchio::JointModel & joint,
    const Vector3d & translation, const std::string & name)
  {
    const auto id = model.addJoint(
      parent, joint, pinocchio::SE3(Matrix3d::Identity(), translation),
      name);
    model.appendBodyToJoint(
      id, pinocchio::Inertia(1, Vector3d(.02, .01, -.08), .01 * Matrix3d::Identity()),
      pinocchio::SE3::Identity());
    return id;
  }
  static double value_for_name(const std::string & name)
  {
    return .02 * (1 + std::accumulate(name.begin(), name.end(), 0) % 13);
  }
  void set_noise()
  {
    params->joint_position_std_.clear();
    params->joint_velocity_std_.clear();
    for (const auto & name : params->joint_names_) {
      params->joint_position_std_.push_back(.01 + value_for_name(name) * .1);
      params->joint_velocity_std_.push_back(.1 + value_for_name(name));
    }
  }
  std::unique_ptr<robot_IEKF> estimator()
  {
    return std::make_unique<robot_IEKF>(model, *data, store, params);
  }
  int column(const std::string & name) const
  {
    return static_cast<int>(std::find(
             params->joint_names_.begin(), params->joint_names_.end(),
             name) -
           params->joint_names_.begin());
  }
};

Matrix3d skew(const Vector3d & value)
{
  Matrix3d result;
  result << 0, -value.z(), value.y(), value.z(), 0, -value.x(), -value.y(), value.x(), 0;
  return result;
}

TEST(IEKF, JointAndModelOrderDoNotChangeMeasurementsOrEstimates)
{
  Robot a({4, 4}, true), b({4, 4}, true, false, true);
  b.params->joint_names_ = {"leg0_0", "leg1_0", "leg0_1", "leg1_1", "leg0_2",
    "leg1_2", "leg0_3", "leg1_3", "arm1", "arm0"};
  b.set_noise();
  auto first = a.estimator(), second = b.estimator();
  IEKFTestAccess::measure(*first);
  IEKFTestAccess::measure(*second);
  EXPECT_TRUE(
    IEKFTestAccess::positions(*first).isApprox(
      IEKFTestAccess::positions(*second),
      1e-12));
  for (const auto & name : a.params->joint_names_) {
    EXPECT_TRUE(
      IEKFTestAccess::jacobian(*first)
      .col(a.column(name))
      .isApprox(IEKFTestAccess::jacobian(*second).col(b.column(name)), 1e-12))
      << name;
  }
  for (int contact = 0; contact < 2; ++contact) {
    EXPECT_TRUE(
      IEKFTestAccess::velocity(
        *first,
        contact).isApprox(IEKFTestAccess::velocity(*second, contact), 1e-12));
    EXPECT_TRUE(
      IEKFTestAccess::position_covariance(*first, contact)
      .isApprox(IEKFTestAccess::position_covariance(*second, contact), 1e-12));
    EXPECT_TRUE(
      IEKFTestAccess::velocity_covariance(*first, contact)
      .isApprox(IEKFTestAccess::velocity_covariance(*second, contact), 1e-12));
  }
  first->initialize();
  second->initialize();
  for (int step = 0; step < 5; ++step) {
    first->update(step);
    second->update(step);
    EXPECT_TRUE(first->x_est_.allFinite());
    EXPECT_TRUE(first->x_est_.isApprox(second->x_est_, 1e-8));
  }
  EXPECT_LE(IEKFTestAccess::history_size(*first), static_cast<size_t>(a.params->N_ + 1));
}

TEST(IEKF, ContactPermutationKeepsFrameFlagAndCovarianceAssociations)
{
  Robot a({3, 5}), b({3, 5});
  a.store->contact_ << 1, 0;
  std::reverse(b.params->contact_names_.begin(), b.params->contact_names_.end());
  b.store->contact_ << 0, 1;
  auto first = a.estimator(), second = b.estimator();
  IEKFTestAccess::measure(*first);
  IEKFTestAccess::measure(*second);
  EXPECT_DOUBLE_EQ(first->contact_(0), second->contact_(1));
  EXPECT_TRUE(
    IEKFTestAccess::positions(*first).topRows(3).isApprox(
      IEKFTestAccess::positions(
        *second).bottomRows(3)));
  EXPECT_TRUE(IEKFTestAccess::velocity(*first, 0).isApprox(IEKFTestAccess::velocity(*second, 1)));
  EXPECT_TRUE(
    IEKFTestAccess::position_covariance(
      *first,
      0).isApprox(IEKFTestAccess::position_covariance(*second, 1)));
  first->initialize();
  second->initialize();
  for (int step = 0; step < 3; ++step) {
    first->update(step);
    second->update(step);
    EXPECT_TRUE(first->x_est_.isApprox(second->x_est_, 1e-8));
  }
}

TEST(IEKF, ArbitraryContactCountAndUnrelatedArmBranch)
{
  Robot robot({2, 4, 3}, true);
  auto estimator = robot.estimator();
  IEKFTestAccess::measure(*estimator);
  EXPECT_EQ(estimator->contact_.size(), 3);
  EXPECT_EQ(IEKFTestAccess::jacobian(*estimator).rows(), 9);
  EXPECT_EQ(IEKFTestAccess::jacobian(*estimator).cols(), 11);
  EXPECT_TRUE(IEKFTestAccess::jacobian(*estimator).col(robot.column("arm0")).isZero(0));
  EXPECT_TRUE(IEKFTestAccess::jacobian(*estimator).col(robot.column("arm1")).isZero(0));
  const MatrixXd original = IEKFTestAccess::positions(*estimator);
  robot.store->joint_states_position_(
    robot.model.joints[robot.model.getJointId("arm0")].idx_q() -
    7) += .5;
  IEKFTestAccess::measure(*estimator);
  EXPECT_TRUE(original.isApprox(IEKFTestAccess::positions(*estimator), 1e-12));
}

TEST(IEKF, ContactFrameDoesNotHaveToFollowTheLastMovingJoint)
{
  Robot robot;
  const auto parent = robot.model.getJointId("leg0_1");
  robot.model.addFrame(
    pinocchio::Frame(
      "mid_chain_contact", parent, pinocchio::SE3(Matrix3d::Identity(), Vector3d(.1, .03, -.2)),
      pinocchio::OP_FRAME),
    false);
  robot.params->contact_names_[0] = "mid_chain_contact";
  robot.data = std::make_unique<pinocchio::Data>(robot.model);
  auto estimator = robot.estimator();
  IEKFTestAccess::measure(*estimator);
  const MatrixXd J = IEKFTestAccess::jacobian(*estimator);
  EXPECT_TRUE(J.block(0, robot.column("leg0_2"), 3, 1).isZero(0));
  EXPECT_TRUE(J.block(0, robot.column("leg0_3"), 3, 1).isZero(0));
  EXPECT_EQ(IEKFTestAccess::support(*estimator)[0].size(), 2u);
  const Vector3d before = IEKFTestAccess::positions(*estimator).topRows(3);
  robot.store->joint_states_position_(
    robot.model.joints[robot.model.getJointId("leg0_3")].idx_q() -
    7) += .5;
  IEKFTestAccess::measure(*estimator);
  EXPECT_TRUE(before.isApprox(IEKFTestAccess::positions(*estimator).topRows(3), 1e-12));
}

TEST(IEKF, JacobianAndVelocityMatchFiniteDifferencesAtFixedContactOffsets)
{
  Robot robot({3, 5}, true, true, true);
  auto estimator = robot.estimator();
  IEKFTestAccess::measure(*estimator);
  const MatrixXd J = IEKFTestAccess::jacobian(*estimator);
  const double epsilon = 1e-6;
  for (const auto & name : robot.params->joint_names_) {
    const int input = robot.model.joints[robot.model.getJointId(name)].idx_q() - 7;
    const double original = robot.store->joint_states_position_(input);
    robot.store->joint_states_position_(input) = original + epsilon;
    IEKFTestAccess::measure(*estimator);
    const MatrixXd plus = IEKFTestAccess::positions(*estimator);
    robot.store->joint_states_position_(input) = original - epsilon;
    IEKFTestAccess::measure(*estimator);
    const MatrixXd minus = IEKFTestAccess::positions(*estimator);
    robot.store->joint_states_position_(input) = original;
    EXPECT_TRUE(((plus - minus) / (2 * epsilon)).isApprox(J.col(robot.column(name)), 1e-7)) << name;
  }
  const VectorXd original = robot.store->joint_states_position_;
  robot.store->joint_states_position_ = original + epsilon * robot.store->joint_states_velocity_;
  IEKFTestAccess::measure(*estimator);
  const MatrixXd plus = IEKFTestAccess::positions(*estimator);
  robot.store->joint_states_position_ = original - epsilon * robot.store->joint_states_velocity_;
  IEKFTestAccess::measure(*estimator);
  const MatrixXd minus = IEKFTestAccess::positions(*estimator);
  robot.store->joint_states_position_ = original;
  IEKFTestAccess::measure(*estimator);
  for (int contact = 0; contact < 2; ++contact) {
    const Vector3d p = IEKFTestAccess::positions(*estimator).block<3, 1>(3 * contact, 0);
    const Vector3d numerical =
      -(plus - minus).block<3, 1>(3 * contact, 0) / (2 * epsilon) - robot.store->omega_b_.cross(p);
    EXPECT_TRUE(numerical.isApprox(IEKFTestAccess::velocity(*estimator, contact), 1e-7));
  }
  const auto & supports = IEKFTestAccess::support(*estimator);
  EXPECT_EQ(supports[0].front(), robot.column("waist"));
  EXPECT_EQ(supports[1].front(), robot.column("waist"));
}

TEST(IEKF, CovarianceMatchesIndependentNoisePropagationAndIsPositiveSemidefinite)
{
  Robot robot({4, 5}, true, true);
  auto estimator = robot.estimator();
  IEKFTestAccess::measure(*estimator);
  for (int contact = 0; contact < 2; ++contact) {
    const MatrixXd J = IEKFTestAccess::jacobian(*estimator).middleRows(3 * contact, 3);
    const Vector3d p = IEKFTestAccess::positions(*estimator).block<3, 1>(3 * contact, 0);
    Matrix3d expected_position = Matrix3d::Zero(), expected_velocity = Matrix3d::Zero();
    for (int joint = 0; joint < J.cols(); ++joint) {
      const Vector3d direction = J.col(joint),
        angular_direction = skew(robot.store->omega_b_) * direction;
      const double q_variance = std::pow(robot.params->joint_position_std_[joint], 2);
      const double v_variance = std::pow(robot.params->joint_velocity_std_[joint], 2);
      expected_position += q_variance * direction * direction.transpose();
      expected_velocity += v_variance * direction * direction.transpose() +
        q_variance * angular_direction * angular_direction.transpose();
    }
    for (int axis = 0; axis < 3; ++axis) {
      const Vector3d direction = skew(p).col(axis);
      expected_velocity += std::pow(
        robot.params->gyro_input_std_[axis],
        2) * direction * direction.transpose();
    }
    const Matrix3d actual_position = IEKFTestAccess::position_covariance(*estimator, contact);
    const Matrix3d actual_velocity = IEKFTestAccess::velocity_covariance(*estimator, contact);
    EXPECT_TRUE(actual_position.isApprox(expected_position, 1e-12));
    EXPECT_TRUE(actual_velocity.isApprox(expected_velocity, 1e-12));
    for (const auto & covariance : {actual_position, actual_velocity}) {
      EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
      Eigen::SelfAdjointEigenSolver<Matrix3d> eigenvalues(covariance);
      EXPECT_GE(eigenvalues.eigenvalues().minCoeff(), -1e-12);
    }
  }
}

TEST(IEKF, CoupledForcesMatchDisjointPerLegSolveAndSharedChainReference)
{
  for (bool shared : {false, true}) {
    Robot robot({4, 4}, true, shared);
    auto estimator = robot.estimator();
    IEKFTestAccess::measure(*estimator);
    const MatrixXd J = IEKFTestAccess::jacobian(*estimator);
    VectorXd residual = VectorXd::Zero(robot.model.nv);
    VectorXd configured(robot.params->joint_names_.size());
    for (int column = 0; column < configured.size(); ++column) {
      configured(column) = column + 1;
      residual(
        robot.model.joints[robot.model.getJointId(
          robot.params->joint_names_[column])].idx_v()) =
        configured(column);
    }
    // QR on the augmented system is independent of the production normal-equation solve.
    MatrixXd augmented = MatrixXd::Zero(configured.size() + J.rows(), J.rows());
    augmented.topRows(configured.size()) = J.transpose();
    augmented.bottomRows(J.rows()) = .01 * MatrixXd::Identity(J.rows(), J.rows());
    VectorXd rhs = VectorXd::Zero(augmented.rows());
    rhs.head(configured.size()) = configured;
    const VectorXd expected = augmented.colPivHouseholderQr().solve(rhs);
    const VectorXd actual = IEKFTestAccess::forces(*estimator, residual);
    EXPECT_TRUE(actual.isApprox(expected, 1e-9));
    if (!shared) {
      for (int contact = 0; contact < 2; ++contact) {
        const MatrixXd leg_J = J.middleRows(3 * contact, 3);
        const Vector3d legacy =
          (leg_J * leg_J.transpose() + 1e-4 * Matrix3d::Identity()).ldlt().solve(
          leg_J *
          configured);
        EXPECT_TRUE(actual.segment<3>(3 * contact).isApprox(legacy, 1e-10));
      }
    }
    residual(robot.model.joints[robot.model.getJointId("arm0")].idx_v()) += 100;
    EXPECT_TRUE(actual.isApprox(IEKFTestAccess::forces(*estimator, residual), 1e-12));
    EXPECT_THROW(IEKFTestAccess::forces(*estimator, VectorXd::Zero(1)), std::invalid_argument);
  }
}

TEST(IEKF, ScalarPerJointAndLegacyNoiseRemainUnambiguous)
{
  Robot robot;
  std::reverse(robot.params->joint_names_.begin(), robot.params->joint_names_.end());
  robot.params->dim_legs_ = 4;
  robot.params->num_legs_ = 2;
  robot.params->joint_position_std_ = {.01, .02, .03, .04};
  robot.params->joint_velocity_std_ = {.2};
  auto estimator = robot.estimator();
  for (int leg = 0; leg < 2; ++leg) {
    for (int depth = 0; depth < 4; ++depth) {
      const int column = robot.column("leg" + std::to_string(leg) + "_" + std::to_string(depth));
      EXPECT_NEAR(
        IEKFTestAccess::position_noise (*estimator)(column, column),
        std::pow(.01 * (depth + 1), 2), 1e-14);
      EXPECT_NEAR(IEKFTestAccess::velocity_noise (*estimator)(column, column), .04, 1e-14);
    }
  }
  robot.params->joint_position_std_ = {.1, .2, .3, .4, .5, .6, .7, .8};
  estimator = robot.estimator();
  EXPECT_NEAR(IEKFTestAccess::position_noise (*estimator)(0, 0), .01, 1e-14);
  robot.params->joint_position_std_ = {.1, .2};
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  for (auto topology : {std::vector<int>{3, 4}, std::vector<int>{4, 4}}) {
    Robot invalid(topology, topology[0] == 4);
    invalid.params->dim_legs_ = 4;
    invalid.params->joint_position_std_ = {.1, .2, .3, .4};
    EXPECT_THROW(invalid.estimator(), std::invalid_argument);
  }
  Robot shared({4, 4}, false, true);
  shared.params->dim_legs_ = 5;
  shared.params->joint_position_std_ = {.1, .2, .3, .4, .5};
  EXPECT_THROW(shared.estimator(), std::invalid_argument);
}

TEST(IEKF, ConfigurationValidationRejectsInvalidNamesDimensionsAndModels)
{
  Robot robot;
  const auto original = robot.params->joint_names_;
  robot.params->joint_names_[0] = "missing";
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->joint_names_ = original;
  robot.params->joint_names_[0] = original[1];
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->joint_names_ = original;
  robot.params->joint_names_.pop_back();
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->joint_names_ = original;
  robot.params->contact_names_ = {"missing"};
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->contact_names_ = {"sole0", "sole0"};
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->contact_names_ = {"sole0", "sole1"};
  robot.params->num_legs_ = 3;
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->num_legs_ = 0;
  robot.params->rate_ = 0;
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->rate_ = 500;
  robot.params->N_ = 0;
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->N_ = 1;
  robot.params->joint_position_std_[0] = -1;
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.params->joint_position_std_[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  robot.set_noise();
  robot.params->gyro_input_std_ = {1, 2};
  EXPECT_THROW(robot.estimator(), std::invalid_argument);
  EXPECT_THROW(robot_IEKF(robot.model, *robot.data, nullptr, robot.params), std::invalid_argument);
  Robot continuous;
  continuous.add_joint(1, pinocchio::JointModelRUBZ(), Vector3d::Zero(), "continuous");
  continuous.data = std::make_unique<pinocchio::Data>(continuous.model);
  continuous.params->joint_names_.push_back("continuous");
  EXPECT_THROW(continuous.estimator(), std::invalid_argument);
  Robot spherical;
  spherical.add_joint(1, pinocchio::JointModelSpherical(), Vector3d::Zero(), "spherical");
  spherical.data = std::make_unique<pinocchio::Data>(spherical.model);
  EXPECT_THROW(spherical.estimator(), std::invalid_argument);
  pinocchio::Model fixed;
  pinocchio::Data fixed_data(fixed);
  EXPECT_THROW(robot_IEKF(fixed, fixed_data, robot.store, robot.params), std::invalid_argument);
}

TEST(IEKF, InvalidMeasurementsDoNotPropagateOrChangePublishedState)
{
  Robot robot;
  auto estimator = robot.estimator();
  estimator->initialize();
  const VectorXd state = estimator->x_est_;
  const MatrixXd covariance = IEKFTestAccess::filter_covariance(*estimator);
  robot.store->joint_states_position_(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(estimator->update(1), std::invalid_argument);
  EXPECT_TRUE(state.isApprox(estimator->x_est_, 0));
  EXPECT_TRUE(covariance.isApprox(IEKFTestAccess::filter_covariance(*estimator), 0));
  robot.store->joint_states_position_(0) = .1;
  robot.store->joint_states_velocity_.resize(1);
  EXPECT_THROW(estimator->update(1), std::invalid_argument);
  robot.store->joint_states_velocity_ = VectorXd::Zero(8);
  robot.store->contact_.resize(1);
  EXPECT_THROW(estimator->update(1), std::invalid_argument);
  robot.store->contact_ = VectorXd::Ones(2);
  robot.store->quaternion_.coeffs().setZero();
  EXPECT_THROW(estimator->update(1), std::invalid_argument);
  robot.store->quaternion_ = Quaterniond::Identity();
  robot.store->accel_b_(0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(estimator->update(1), std::invalid_argument);
}

TEST(IEKF, MomentumObserverIsOptionalAndReconstructionIsDeterministic)
{
  Robot robot;
  robot.store->joint_states_effort_.resize(0);
  auto estimator = robot.estimator();
  EXPECT_FALSE(IEKFTestAccess::has_observer(*estimator));
  estimator->initialize();
  EXPECT_TRUE(estimator->x_est_.allFinite());
  robot.params->using_mob_ = 1;
  robot.store->joint_states_effort_ = VectorXd::Zero(8);
  // External flags are unused in momentum mode.
  robot.store->contact_.resize(0);
  VectorXd first_state, first_flags;
  for (int repetition = 0; repetition < 5; ++repetition) {
    estimator = robot.estimator();
    EXPECT_TRUE(IEKFTestAccess::has_observer(*estimator));
    estimator->initialize();
    EXPECT_TRUE(estimator->x_est_.allFinite());
    if (repetition == 0) {
      first_state = estimator->x_est_;
      first_flags = estimator->contact_;
    } else {
      EXPECT_TRUE(first_state.isApprox(estimator->x_est_, 1e-12));
      EXPECT_TRUE(first_flags.isApprox(estimator->contact_, 0));
    }
  }
  robot.store->joint_states_effort_(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(estimator->update(1), std::invalid_argument);
}
}  // namespace
