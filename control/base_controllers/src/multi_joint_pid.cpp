#include "base_controllers/multi_joint_pid.hpp"

namespace base_controllers
{
void MultiJointPID::init(size_t n)
{
  if (n == 0)
  {
    throw std::invalid_argument("MultiJointPID::init requires n > 0");
  }
  pid_controllers_.resize(n);
  outputs_.assign(n, 0.0);
  initialized_ = true;
}

void MultiJointPID::ensure_initialized(size_t idx) const
{
  if (!initialized_)
  {
    throw std::runtime_error("MultiJointPID not initialized. Call init(n) before using.");
  }
  if (idx >= pid_controllers_.size())
  {
    throw std::out_of_range("Index " + std::to_string(idx) + " out of range for MultiJointPID of size " +
                            std::to_string(pid_controllers_.size()));
  }
}

void MultiJointPID::set_gains(size_t idx, double p, double i, double d, double i_max, double i_min)
{
  ensure_initialized(idx);
  pid_controllers_[idx].initPid(p, i, d, i_max, i_min);
}

void MultiJointPID::set_gains(const std::vector<double>& p, const std::vector<double>& i, const std::vector<double>& d,
                              double i_max, double i_min)
{
  if (!initialized_)
    throw std::runtime_error("MultiJointPID not initialized");
  if (p.size() != pid_controllers_.size() || i.size() != pid_controllers_.size() || d.size() != pid_controllers_.size())
  {
    throw std::invalid_argument("gain vector sizes must match number of controllers");
  }
  for (size_t idx = 0; idx < pid_controllers_.size(); ++idx)
  {
    pid_controllers_[idx].initPid(p[idx], i[idx], d[idx], i_max, i_min);
  }
}

double MultiJointPID::compute(size_t idx, double error, const rclcpp::Duration& dt)
{
  ensure_initialized(idx);
  // if (dt <= 0.0) return outputs_[idx];

  uint64_t dt_ns = dt.nanoseconds();
  double out = pid_controllers_[idx].computeCommand(error, dt_ns);
  outputs_[idx] = out;
  return out;
}

std::vector<double> MultiJointPID::compute(const std::vector<double>& errors, const rclcpp::Duration& dt)
{
  if (!initialized_)
    throw std::runtime_error("MultiJointPID not initialized");
  if (errors.size() != pid_controllers_.size())
    throw std::invalid_argument("errors size mismatch");
  // if (dt <= 0.0) return outputs_;

  uint64_t dt_ns = dt.nanoseconds();
  for (size_t idx = 0; idx < pid_controllers_.size(); ++idx)
  {
    outputs_[idx] = pid_controllers_[idx].computeCommand(errors[idx], dt_ns);
  }
  return outputs_;
}

double MultiJointPID::compute(size_t idx, double error, double error_dot, const rclcpp::Duration& dt)
{
  ensure_initialized(idx);
  // if (dt <= 0.0) return outputs_[idx];

  uint64_t dt_ns = dt.nanoseconds();
  double out = pid_controllers_[idx].computeCommand(error, error_dot, dt_ns);
  outputs_[idx] = out;
  return out;
}

std::vector<double> MultiJointPID::compute(const std::vector<double>& errors, const std::vector<double>& errors_dot,
                                           const rclcpp::Duration& dt)
{
  if (!initialized_)
    throw std::runtime_error("MultiJointPID not initialized");
  if (errors.size() != pid_controllers_.size())
    throw std::invalid_argument("errors size mismatch");
  // if (dt <= 0.0) return outputs_;

  uint64_t dt_ns = dt.nanoseconds();
  for (size_t idx = 0; idx < pid_controllers_.size(); ++idx)
  {
    outputs_[idx] = pid_controllers_[idx].computeCommand(errors[idx], errors_dot[idx], dt_ns);
  }
  return outputs_;
}

void MultiJointPID::reset(bool clear_outputs)
{
  if (!initialized_)
    return;
  for (auto& pid : pid_controllers_)
  {
    pid.reset();
  }
  if (clear_outputs)
  {
    std::fill(outputs_.begin(), outputs_.end(), 0.0);
  }
}

void MultiJointPID::cleanup()
{
  pid_controllers_.clear();
  outputs_.clear();
  initialized_ = false;
}

};  // namespace base_controllers
