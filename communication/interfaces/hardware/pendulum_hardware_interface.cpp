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
        LowState_subscriber_ = node_ptr_->create_subscription<pendulum_msgs::msg::LowState>(
            "/pendulum/sensor_state", 10,
            [this](const pendulum_msgs::msg::LowState::SharedPtr msg) {
                    pivot_position_ = msg->motor_state.q;
                    pivot_velocity_ = msg->motor_state.dq;
                    pivot_effort_ = msg->motor_state.tau;
            });
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
        state_interfaces.emplace_back("tip_sensor", "y", &tip_sensor_y_);
        state_interfaces.emplace_back("tip_sensor", "z", &tip_sensor_z_);
        state_interfaces.emplace_back("tip_sensor", "vy", &tip_sensor_vy_);
        state_interfaces.emplace_back("tip_sensor", "vz", &tip_sensor_vz_);
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> PendulumHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back("pivot", "effort", &pivot_effort_command_);
        return command_interfaces;
    }

    hardware_interface::return_type PendulumHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // TODO: implement reading from hardware
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PendulumHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // TODO: implement writing to hardware
        return hardware_interface::return_type::OK;
    }

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware_interfaces::PendulumHardwareInterface, 
            hardware_interface::SystemInterface)