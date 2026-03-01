#include "../include/end_effectors/gripper_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
GripperHardwareInterface::GripperHardwareInterface() : EndEffectorHardwareInterface()
{
}
hardware_interface::CallbackReturn GripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // create the node
  node_ptr_ = rclcpp::Node::make_shared(node_name_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
  GripperState_subscriber_ = node_ptr_->create_subscription<end_effector_msgs::msg::GripperState>(
      subscribe_topic_name_, qos,
      [this](const end_effector_msgs::msg::GripperState::SharedPtr msg)
      {
        // assume another joint mimic current joint
        joint_pos_[0] = msg->gripper_state.q;
        joint_vel_[0] = msg->gripper_state.dq;
        joint_acc_[0] = msg->gripper_state.ddq;
        joint_tau_[0] = msg->gripper_state.tau;
      });
  GripperCmd_publisher_ = node_ptr_->create_publisher<end_effector_msgs::msg::GripperCmd>(publish_topic_name_, qos);
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
  if (info_.joints.size() != 1 && info_.joints.size() != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GripperHardwareInterface"), "Expected exactly one or two joints, but got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto ret = EndEffectorHardwareInterface::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
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
    // assume another joint mimic current joint
    auto& msg = realtime_GripperCmd_publisher_->msg_;
    msg.gripper_cmd.mode = mode_[0];
    msg.gripper_cmd.q = joint_pos_command_[0];
    msg.gripper_cmd.dq = joint_vel_command_[0];
    msg.gripper_cmd.tau = joint_tau_command_[0];
    msg.gripper_cmd.kp = joint_kp_[0];
    msg.gripper_cmd.kd = joint_kd_[0];
    realtime_GripperCmd_publisher_->unlockAndPublish();
  }
  return hardware_interface::return_type::OK;
}

}; // namespace hardware_interfaces
