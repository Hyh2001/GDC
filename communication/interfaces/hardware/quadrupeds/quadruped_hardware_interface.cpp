#include "../include/quadrupeds/quadruped_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
QuadrupedHardwareInterface::QuadrupedHardwareInterface() : hardware_interface::SystemInterface()
{
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  // create the node
  node_ptr_ = rclcpp::Node::make_shared(node_name_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
  LowState_subscriber_ = node_ptr_->create_subscription<quadruped_msgs::msg::LowState>(
      subscribe_topic_name_, qos,
      [this](const quadruped_msgs::msg::LowState::SharedPtr msg)
      {
        for (int i = 0; i < 12; i++)
        {
          joint_pos_[i] = msg->motor_state[i].q;
          joint_vel_[i] = msg->motor_state[i].dq;
          joint_acc_[i] = msg->motor_state[i].ddq;
          joint_tau_[i] = msg->motor_state[i].tau;
        }
        for (int i = 0; i < 4; i++)
        {
          contact_states_[i] = msg->contact_state[i].contact;
        }
        gyro_[0] = msg->imu.angular_velocity.x;
        gyro_[1] = msg->imu.angular_velocity.y;
        gyro_[2] = msg->imu.angular_velocity.z;
        accel_[0] = msg->imu.linear_acceleration.x;
        accel_[1] = msg->imu.linear_acceleration.y;
        accel_[2] = msg->imu.linear_acceleration.z;
      });
  LowCmd_publisher_ = node_ptr_->create_publisher<quadruped_msgs::msg::LowCmd>(publish_topic_name_, qos);
  realtime_LowCmd_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<quadruped_msgs::msg::LowCmd>>(LowCmd_publisher_);
  executor_.add_node(node_ptr_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  info_ = info;

  return hardware_interface::SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> QuadrupedHardwareInterface::export_state_interfaces()
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
      else if (iface.name == "effort")
      {
        state_interfaces.emplace_back(info_.joints[i].name, iface.name, &joint_tau_[i]);
      }
    }
  }
  for (size_t i = 0; i < info_.sensors.size(); ++i)
  {
    if (info_.sensors[i].name == "imu")
    {
      for (const auto& iface : info_.sensors[i].state_interfaces)
      {
        if (iface.name == "angular_velocity.x")
        {
          state_interfaces.emplace_back(info_.sensors[i].name, iface.name, &gyro_[0]);
        }
        else if (iface.name == "angular_velocity.y")
        {
          state_interfaces.emplace_back(info_.sensors[i].name, iface.name, &gyro_[1]);
        }
        else if (iface.name == "angular_velocity.z")
        {
          state_interfaces.emplace_back(info_.sensors[i].name, iface.name, &gyro_[2]);
        }
        else if (iface.name == "linear_acceleration.x")
        {
          state_interfaces.emplace_back(info_.sensors[i].name, iface.name, &accel_[0]);
        }
        else if (iface.name == "linear_acceleration.y")
        {
          state_interfaces.emplace_back(info_.sensors[i].name, iface.name, &accel_[1]);
        }
        else if (iface.name == "linear_acceleration.z")
        {
          state_interfaces.emplace_back(info_.sensors[i].name, iface.name, &accel_[2]);
        }
      }
    }
    else if (info_.sensors[i].name == "contact_sensor")
    {
      for (size_t j = 0; j < info_.sensors[i].state_interfaces.size(); ++j)
      {
        state_interfaces.emplace_back(info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name,
                                      &contact_states_[j]);
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> QuadrupedHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto& iface : info_.joints[i].command_interfaces)
    {
      if (iface.name == "position")
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

hardware_interface::return_type QuadrupedHardwareInterface::read(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& period)
{
  executor_.spin_some(std::chrono::milliseconds(1));
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuadrupedHardwareInterface::write(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  auto msg = quadruped_msgs::msg::LowCmd();
  msg.header.stamp = node_ptr_->now();
  for (int i = 0; i < 12; i++)
  {
    msg.motor_cmd[i].mode = mode_[i];
    msg.motor_cmd[i].q = joint_pos_command_[i];
    msg.motor_cmd[i].dq = joint_vel_command_[i];
    msg.motor_cmd[i].tau = joint_tau_command_[i];
    msg.motor_cmd[i].kp = joint_kp_[i];
    msg.motor_cmd[i].kd = joint_kd_[i];
  }
  // publish the command
  realtime_LowCmd_publisher_->lock();
  realtime_LowCmd_publisher_->msg_ = msg;
  realtime_LowCmd_publisher_->unlockAndPublish();
  return hardware_interface::return_type::OK;
}

};  // namespace hardware_interfaces
