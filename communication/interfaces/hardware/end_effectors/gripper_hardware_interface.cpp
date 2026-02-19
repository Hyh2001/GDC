#include "../include/end_effectors/gripper_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
hardware_interface::CallbackReturn GripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // create the node
  node_ptr_ = rclcpp::Node::make_shared(node_name_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
  GripperState_subscriber_ = node_ptr_->create_subscription<end_effector_msgs::msg::GripperState>(
      subscribe_topic_name_, qos,
      [this](const end_effector_msgs::msg::GripperState::SharedPtr msg)
      {
        for (size_t i = 0; i < joint_pos_.size(); i++)
        {
          joint_pos_[i] = msg->motor_state[i].q;
          joint_vel_[i] = msg->motor_state[i].dq;
          joint_acc_[i] = msg->motor_state[i].ddq;
          joint_tau_[i] = msg->motor_state[i].tau;
        }
      });
  GripperCmd_publisher_ = node_ptr_->create_publisher<end_effector_msgs::msg::GripperState>(publish_topic_name_, qos);
  realtime_GripperCmd_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<end_effector_msgs::msg::GripperCmd>>(GripperCmd_publisher_);
  executor_.add_node(node_ptr_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  info_ = info;
  // resize
  if (info_.joints.size() != 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GripperHardwareInterface"), "Expected exactly one joint, but got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  EndEffectorHardwareInterface::on_init(info);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperHardwareInterface::export_state_interfaces()
{
  return EndEffectorHardwareInterface::export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface> GripperHardwareInterface::export_command_interfaces()
{
  return EndEffectorHardwareInterface::export_command_interfaces();
}

hardware_interface::return_type GripperHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  executor_.spin_some(std::chrono::milliseconds(1));
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (realtime_GripperCmd_publisher_ && realtime_GripperCmd_publisher_->trylock())
  {
    auto& msg = realtime_GripperCmd_publisher_->msg_;
    for (size_t i = 0; i < joint_pos_command_.size(); ++i)
    {
      msg.motor_cmd[i].mode = mode_[i];
      msg.motor_cmd[i].q = joint_pos_command_[i];
      msg.motor_cmd[i].dq = joint_vel_command_[i];
      msg.motor_cmd[i].tau = joint_tau_command_[i];
      msg.motor_cmd[i].kp = joint_kp_[i];
      msg.motor_cmd[i].kd = joint_kd_[i];
    }
    realtime_GripperCmd_publisher_->unlockAndPublish();
  }
  return hardware_interface::return_type::OK;
}

}; // namespace hardware_interfaces
