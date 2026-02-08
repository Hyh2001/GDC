#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

#include "control_toolbox/pid.hpp"

namespace base_controllers
{

class MultiJointPID
{
  public:
  MultiJointPID() = default;

  void init(size_t n);

  void set_gains(size_t idx, double p, double i, double d, double i_max = 1e6, double i_min = -1e6);  // for single PID
  void set_gains(const std::vector<double>& p, const std::vector<double>& i, const std::vector<double>& d,
                 double i_max = 1e6, double i_min = -1e6);  // for multiple PIDs

  double compute(size_t idx, double error, const rclcpp::Duration& dt);                        // for single PID
  std::vector<double> compute(const std::vector<double>& errors, const rclcpp::Duration& dt);  // for multiple PIDs

  double compute(size_t idx, double error, double error_dot, const rclcpp::Duration& dt);  // for single PID
  std::vector<double> compute(const std::vector<double>& errors, const std::vector<double>& errors_dot,
                              const rclcpp::Duration& dt);  // for multiple PIDs

  void reset(bool clear_outputs = true);
  void cleanup();

  const double& getOutput(size_t idx) const
  {
    ensure_initialized(idx);
    return outputs_[idx];
  }

  const std::vector<double>& getOutputs() const
  {
    return outputs_;
  }

  protected:
  bool initialized_{false};
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> outputs_;

  void ensure_initialized(size_t idx) const;
};

};  // namespace base_controllers
