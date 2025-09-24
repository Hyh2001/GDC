#pragma once

#include <vector>
#include <string>
#include <stdexcept>

class PID
{
public:
    PID() = default;

    PID(int num_joints,
        const std::vector<double> &Kp,
        const std::vector<double> &Ki,
        const std::vector<double> &Kd)
    {
        num_joints_ = num_joints;
        configure(Kp, Ki, Kd);
    }

    void configure(const std::vector<double> &Kp,
                   const std::vector<double> &Ki,
                   const std::vector<double> &Kd)
    {
        if (Kp.size() != num_joints_ || Kd.size() != num_joints_ || Ki.size() != num_joints_)
        {
            throw std::invalid_argument("PID gain vectors must be the same size.");
        }

        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;

        integral_.assign(Kp.size(), 0.0);
        prev_error_.assign(Kp.size(), 0.0);
    }

    /// Compute control output for all joints
    std::vector<double> compute(const std::vector<double> &desired,
                                const std::vector<double> &measured,
                                double dt)
    {
        if (desired.size() != measured.size() || desired.size() != num_joints_)
        {
            throw std::invalid_argument("Invalid joint targets or measurements.");
        }

        std::vector<double> output(desired.size());

        for (size_t i = 0; i < desired.size(); i++)
        {
            double error = desired[i] - measured[i];
            // Integral
            integral_[i] += error * dt;
            // Derivative
            double derivative = (error - prev_error_[i]) / dt;
            // PID control law
            output[i] = Kp_[i] * error + Ki_[i] * integral_[i] + Kd_[i] * derivative;
            prev_error_[i] = error;
        }

        return output;
    }

    void reset()
    {
        std::fill(integral_.begin(), integral_.end(), 0.0);
        std::fill(prev_error_.begin(), prev_error_.end(), 0.0);
    }

private:
    int num_joints_{0};
    std::vector<double> Kp_, Ki_, Kd_;
    std::vector<double> integral_;
    std::vector<double> prev_error_;
};
