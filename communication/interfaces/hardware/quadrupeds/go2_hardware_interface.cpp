#include "../include/quadrupeds/go2_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
    Go2HardwareInterface::Go2HardwareInterface()
    : QuadrupedHardwareInterface()
    {
        node_name_ = "go2_hardware_interface";
        publish_topic_name_ = "/go2/low_cmd";
        subscribe_topic_name_ = "/go2/low_state";

        mode_.fill(4);
    }

    hardware_interface::CallbackReturn Go2HardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        return QuadrupedHardwareInterface::on_configure(previous_state);
    }

    hardware_interface::CallbackReturn Go2HardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Go2HardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn Go2HardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Go2HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Go2HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        return QuadrupedHardwareInterface::on_init(info);
    }

    std::vector<hardware_interface::StateInterface> Go2HardwareInterface::export_state_interfaces() {
        return QuadrupedHardwareInterface::export_state_interfaces();
    }

    std::vector<hardware_interface::CommandInterface> Go2HardwareInterface::export_command_interfaces() {
        return QuadrupedHardwareInterface::export_command_interfaces();
    }

    hardware_interface::return_type Go2HardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return QuadrupedHardwareInterface::read(time, period);
    }

    hardware_interface::return_type Go2HardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return QuadrupedHardwareInterface::write(time, period);
    }

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware_interfaces::Go2HardwareInterface,
            hardware_interface::SystemInterface)
