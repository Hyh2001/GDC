#include "base_controllers/multi_joint_pid.hpp"

namespace base_controllers {
    void MultiJointPID::init(size_t n)
    {
        if (n == 0) {
            throw std::invalid_argument("MultiJointPID::init requires n > 0");
        }
        pid_controllers_.resize(n);
        outputs_.assign(n, 0.0);
        initialized_ = true;
    }

    void MultiJointPID::ensure_initialized(size_t idx) const
    {
        if (!initialized_) {
            throw std::runtime_error("MultiJointPID not initialized. Call init(n) before using.");
        }
        if (idx >= pid_controllers_.size()) {
            throw std::out_of_range("Index " + std::to_string(idx) + " out of range for MultiJointPID of size " + std::to_string(pid_controllers_.size()));
        }
    }

    void MultiJointPID::set_gains(size_t idx, double p, double i, double d,
                   double i_max, double i_min)
    {
        ensure_initialized(idx);
        pid_controllers_[idx].initPid(p, i, d, i_max, i_min);
    }

    void MultiJointPID::set_gains(const std::vector<double> &p, const std::vector<double> &i, const std::vector<double> &d,
                   double i_max, double i_min)
    {
        if (!initialized_) throw std::runtime_error("MultiJointPID not initialized");
        if (p.size() != pid_controllers_.size() ||
            i.size() != pid_controllers_.size() ||
            d.size() != pid_controllers_.size()) {
            throw std::invalid_argument("gain vector sizes must match number of controllers");
        }
        for (size_t idx = 0; idx < pid_controllers_.size(); ++idx) {
            pid_controllers_[idx].initPid(p[idx], i[idx], d[idx], i_max, i_min);
        }
    }

    double MultiJointPID::compute(size_t idx, double error, uint64_t dt)
    {
        ensure_initialized(idx);
        // if (dt <= 0.0) return outputs_[idx];

        // rclcpp::Duration dur = rclcpp::Duration::from_seconds(dt);
        double out = pid_controllers_[idx].computeCommand(error, dt);
        outputs_[idx] = out;
        return out;
    }

    std::vector<double> MultiJointPID::compute(const std::vector<double> &errors, uint64_t dt)
    {
        if (!initialized_) throw std::runtime_error("MultiJointPID not initialized");
        if (errors.size() != pid_controllers_.size()) throw std::invalid_argument("errors size mismatch");
        // if (dt <= 0.0) return outputs_;

        // rclcpp::Duration dur = rclcpp::Duration::from_seconds(dt);
        for (size_t idx = 0; idx < pid_controllers_.size(); ++idx) {
            outputs_[idx] = pid_controllers_[idx].computeCommand(errors[idx], dt);
        }
        return outputs_;
    }

    void MultiJointPID::reset(bool clear_outputs)
    {
        if (!initialized_) return;
        for (auto &pid : pid_controllers_) {
            pid.reset();
        }
        if (clear_outputs) {
            std::fill(outputs_.begin(), outputs_.end(), 0.0);
        }
    }


}; 