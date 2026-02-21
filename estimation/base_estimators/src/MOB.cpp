#include "base_estimators/MOB.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>

MomentumObserver::MomentumObserver(
    const pinocchio::Model &model,
    double dt)
    : model_(model),
      data_(model),
      dt_(dt)
{
    const int nv = model_.nv;

    p0_ = Eigen::VectorXd::Zero(nv);
    p_hat_ = Eigen::VectorXd::Zero(nv);
    r_ = Eigen::VectorXd::Zero(nv);

    K_ = 80.0 * Eigen::MatrixXd::Identity(nv, nv);
}

void MomentumObserver::setGain(double gain)
{
    K_ = gain * Eigen::MatrixXd::Identity(model_.nv, model_.nv);
}

void MomentumObserver::init(const Eigen::VectorXd &q,
                            const Eigen::VectorXd &v)
{
    // Compute mass matrix
    pinocchio::crba(model_, data_, q);
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Initial generalized momentum
    p0_ = data_.M * v;

    // Initialize internal state
    p_hat_ = p0_;
    r_.setZero();
}

void MomentumObserver::update(
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &v,
    const Eigen::VectorXd &tau)
{
    // Compute all required dynamics terms
    pinocchio::computeAllTerms(model_, data_, q, v);

    // Ensure mass matrix symmetry
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Current generalized momentum
    Eigen::VectorXd p = data_.M * v;

    // Nonlinear effects (C*v + g)
    Eigen::VectorXd nle = data_.nle;

    // Compute gravity explicitly
    pinocchio::computeGeneralizedGravity(model_, data_, q);
    Eigen::VectorXd g = data_.g;

    // Extract C*v
    Eigen::VectorXd Cv = nle - g;

    // Observer integration
    p_hat_ += (Cv - g + tau + r_) * dt_;

    // Residual update
    r_ = K_ * (p - p_hat_ - p0_);
}

const Eigen::VectorXd &MomentumObserver::getResidual() const
{
    return r_;
}
