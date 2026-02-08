#include "include/pendulum_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
    PendulumHardwareInterface::PendulumHardwareInterface()
    : hardware_interface::SystemInterface()
    {
    }

    hardware_interface::CallbackReturn PendulumHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        // create the node
        node_ptr_ = rclcpp::Node::make_shared("pendulum_hardware_interface");
        auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
        LowState_subscriber_ = node_ptr_->create_subscription<pend_msgs::msg::LowState>(
            "/pendulum/low_state", qos,
            [this](const pend_msgs::msg::LowState::SharedPtr msg) {
                    pivot_position_ = msg->motor_state.q;
                    pivot_velocity_ = msg->motor_state.dq;
                    pivot_effort_ = msg->motor_state.tau;
            });
        LowCmd_publisher_ = node_ptr_->create_publisher<pend_msgs::msg::LowCmd>("/pendulum/low_cmd", 10);
        realtime_LowCmd_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<pend_msgs::msg::LowCmd>>(LowCmd_publisher_);
        executor_.add_node(node_ptr_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn PendulumHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PendulumHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back("pivot", "position", &pivot_position_);
        state_interfaces.emplace_back("pivot", "velocity", &pivot_velocity_);
        state_interfaces.emplace_back("pivot", "effort", &pivot_effort_);
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> PendulumHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back("pivot", "position", &pivot_pos_command_);
        command_interfaces.emplace_back("pivot", "velocity", &pivot_vel_command_);
        command_interfaces.emplace_back("pivot", "effort", &pivot_effort_command_);
        command_interfaces.emplace_back("pivot", "kp", &kp_command_);
        command_interfaces.emplace_back("pivot", "kd", &kd_command_);
        return command_interfaces;
    }

    hardware_interface::return_type PendulumHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        executor_.spin_some(std::chrono::milliseconds(1));
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PendulumHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        auto msg = pend_msgs::msg::LowCmd();
        msg.header.stamp = node_ptr_->now();
        msg.motor_cmd.kp = kp_command_;
        msg.motor_cmd.kd = kd_command_;
        msg.motor_cmd.q = pivot_pos_command_;
        msg.motor_cmd.dq = pivot_vel_command_;
        msg.motor_cmd.tau = pivot_effort_command_;
        realtime_LowCmd_publisher_->lock();
        realtime_LowCmd_publisher_->msg_ = msg;
        realtime_LowCmd_publisher_->unlockAndPublish();
        // LowCmd_publisher_->publish(msg);
        return hardware_interface::return_type::OK;
    }

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware_interfaces::PendulumHardwareInterface,
            hardware_interface::SystemInterface)
