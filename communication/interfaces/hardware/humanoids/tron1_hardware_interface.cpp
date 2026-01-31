#include "../include/humanoids/tron1_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
    Tron1HardwareInterface::Tron1HardwareInterface()
    : HumanoidHardwareInterface()
    {
        node_name_ = "tron1_hardware_interface";
        publish_topic_name_ = "/tron1/low_cmd";
        subscribe_topic_name_ = "/tron1/low_state";
    }

    hardware_interface::CallbackReturn Tron1HardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
    {           
        return HumanoidHardwareInterface::on_configure(previous_state);
    }        

    hardware_interface::CallbackReturn Tron1HardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Tron1HardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Tron1HardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Tron1HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Tron1HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        auto ret = HumanoidHardwareInterface::on_init(info);
        if (ret != hardware_interface::CallbackReturn::SUCCESS)
        {
            return ret;
        }
        std::fill(mode_.begin(), mode_.end(), 4); // torque + pd
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Tron1HardwareInterface::export_state_interfaces() {
        return HumanoidHardwareInterface::export_state_interfaces();
    }

    std::vector<hardware_interface::CommandInterface> Tron1HardwareInterface::export_command_interfaces() {
        return HumanoidHardwareInterface::export_command_interfaces();
    }

    hardware_interface::return_type Tron1HardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return HumanoidHardwareInterface::read(time, period);
    }

    hardware_interface::return_type Tron1HardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return HumanoidHardwareInterface::write(time, period);
    }

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware_interfaces::Tron1HardwareInterface, 
            hardware_interface::SystemInterface)