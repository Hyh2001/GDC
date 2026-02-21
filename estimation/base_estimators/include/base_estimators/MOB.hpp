#pragma once
#ifndef MOB_HPP
#define MOB_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

class MomentumObserver
{
public:
    MomentumObserver(const pinocchio::Model& model, double dt);

    // Initialize observer (call once)
    void init(const Eigen::VectorXd& q,
              const Eigen::VectorXd& v);

    // Update observer (call every timestep)
    void update(const Eigen::VectorXd& q,
                const Eigen::VectorXd& v,
                const Eigen::VectorXd& tau);

    // Get disturbance residual J^T λ
    const Eigen::VectorXd& getResidual() const;

    // Optional: change gain
    void setGain(double gain);

private:
    const pinocchio::Model& model_;
    pinocchio::Data data_;

    double dt_;

    Eigen::VectorXd p0_;     // initial momentum
    Eigen::VectorXd p_hat_;  // internal observer state
    Eigen::VectorXd r_;      // disturbance residual
    Eigen::MatrixXd K_;      // observer gain
};

#endif
