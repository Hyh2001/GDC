#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "iekf_estimator/IEKF.hpp"

namespace iekf_estimator
{
// Concrete robot packages inherit this base and register their own plugin.
class IEKFEstimator : public controller_interface::ChainableControllerInterface
{
  public:
  using CallbackReturn = controller_interface::CallbackReturn;
  using return_type = controller_interface::return_type;

  CallbackReturn on_init() final;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) final;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) final;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) final;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) final;
  controller_interface::InterfaceConfiguration command_interface_configuration() const final;
  controller_interface::InterfaceConfiguration state_interface_configuration() const final;

  protected:
  virtual CallbackReturn on_estimator_init()
  {
    return CallbackReturn::SUCCESS;
  }
  // Called after common parameters, model, and sensor buffers are ready.
  virtual CallbackReturn on_estimator_configure(const rclcpp_lifecycle::State&) = 0;
  virtual CallbackReturn on_estimator_activate(const rclcpp_lifecycle::State&)
  {
    return CallbackReturn::SUCCESS;
  }
  virtual CallbackReturn on_estimator_deactivate(const rclcpp_lifecycle::State&)
  {
    return CallbackReturn::SUCCESS;
  }
  virtual CallbackReturn on_estimator_cleanup(const rclcpp_lifecycle::State&)
  {
    return CallbackReturn::SUCCESS;
  }
  // Input buffers contain this cycle's hardware measurements, already in core conventions.
  virtual return_type prepare_estimator_inputs(const rclcpp::Time&, const rclcpp::Duration&)
  {
    return return_type::OK;
  }

  return_type update_reference_from_subscribers(const rclcpp::Time&, const rclcpp::Duration&) final
  {
    return return_type::OK;
  }
  return_type update_and_write_commands(const rclcpp::Time&, const rclcpp::Duration&) final;
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() final
  {
    return {};
  }

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::shared_ptr<robot_params> parameters_;
  std::shared_ptr<robot_store> inputs_;
  std::unique_ptr<robot_IEKF> estimator_;

  private:
  void load_parameters();
  void configure_interfaces();
  void bind_interfaces();
  void read_interfaces(const rclcpp::Time& time);
  void prepare_output();
  void invalidate_outputs();
  void release_estimator();
  return_type fail_update(const char* reason);

  struct JointInput
  {
    size_t position, velocity, effort;
    int input_q, input_v;
  };
  std::vector<JointInput> joint_inputs_;
  std::vector<std::string> requested_interfaces_;
  std::vector<size_t> bound_interfaces_;
  std::array<size_t, 10> imu_inputs_{};  // quaternion wxyz, gyro xyz, acceleration xyz
  std::vector<size_t> contact_inputs_;
  std::vector<std::string> output_names_;
  std::vector<double> outputs_, pending_outputs_;
  Eigen::Matrix3d rotation_body_from_imu_ = Eigen::Matrix3d::Identity();
  bool configured_{false}, active_{false}, faulted_{true};
};
}  // namespace iekf_estimator
