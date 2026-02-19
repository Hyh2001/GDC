#include "../include/manipulators/manipulator_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // create the node
  node_ptr_ = rclcpp::Node::make_shared(node_name_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
  LowState_subscriber_ = node_ptr_->create_subscription<manipulator_msgs::msg::LowState>(
      subscribe_topic_name_, qos,
      [this](const manipulator_msgs::msg::LowState::SharedPtr msg)
      {
        for (size_t i = 0; i < joint_pos_.size(); i++)
        {
          joint_pos_[i] = msg->motor_state[i].q;
          joint_vel_[i] = msg->motor_state[i].dq;
          joint_acc_[i] = msg->motor_state[i].ddq;
          joint_tau_[i] = msg->motor_state[i].tau;
        }
      });
  LowCmd_publisher_ = node_ptr_->create_publisher<manipulator_msgs::msg::LowCmd>(publish_topic_name_, qos);
  realtime_LowCmd_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<manipulator_msgs::msg::LowCmd>>(LowCmd_publisher_);
  executor_.add_node(node_ptr_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  info_ = info;
  size_t num_joints = info_.joints.size();
  joint_pos_.resize(num_joints, 0.0);
  joint_vel_.resize(num_joints, 0.0);
  joint_acc_.resize(num_joints, 0.0);
  joint_tau_.resize(num_joints, 0.0);
  mode_.resize(num_joints, 0);
  joint_pos_command_.resize(num_joints, 0.0);
  joint_vel_command_.resize(num_joints, 0.0);
  joint_tau_command_.resize(num_joints, 0.0);
  joint_kp_.resize(num_joints, 0.0);
  joint_kd_.resize(num_joints, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ManipulatorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto& iface : info_.joints[i].state_interfaces)
    {
      if (iface.name == "position")
      {
        state_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_pos_[i]);
      }
      else if (iface.name == "velocity")
      {
        state_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_vel_[i]);
      }
      else if (iface.name == "acceleration")
      {
        state_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_acc_[i]);
      }
      else if (iface.name == "effort")
      {
        state_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_tau_[i]);
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ManipulatorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto& iface : info_.joints[i].command_interfaces)
    {
      if (iface.name == "mode")
      {
        command_interfaces.emplace_back(info_.joints[i].name, iface.name, &mode_[i]);
      }
      else if (iface.name == "position")
      {
        command_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_pos_command_[i]);
      }
      else if (iface.name == "velocity")
      {
        command_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_vel_command_[i]);
      }
      else if (iface.name == "effort")
      {
        command_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_tau_command_[i]);
      }
      else if (iface.name == "kp")
      {
        command_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_kp_[i]);
      }
      else if (iface.name == "kd")
      {
        command_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_kd_[i]);
      }
    }
  }
  return command_interfaces;
}

hardware_interface::return_type ManipulatorHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  executor_.spin_some(std::chrono::milliseconds(1));
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ManipulatorHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (realtime_LowCmd_publisher_ && realtime_LowCmd_publisher_->trylock())
  {
    auto& msg = realtime_LowCmd_publisher_->msg_;
    for (size_t i = 0; i < joint_pos_command_.size(); ++i)
    {
      msg.motor_cmd[i].mode = static_cast<uint8_t>(mode_[i]);
      msg.motor_cmd[i].q = joint_pos_command_[i];
      msg.motor_cmd[i].dq = joint_vel_command_[i];
      msg.motor_cmd[i].tau = joint_tau_command_[i];
      msg.motor_cmd[i].kp = joint_kp_[i];
      msg.motor_cmd[i].kd = joint_kd_[i];
    }
    realtime_LowCmd_publisher_->unlockAndPublish();
  }
  return hardware_interface::return_type::OK;
}

}; // namespace hardware_interfaces
