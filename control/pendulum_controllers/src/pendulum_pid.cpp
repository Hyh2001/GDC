#include "pendulum_controllers/pendulum_pid.hpp"

namespace pendulum_controllers
{

    controller_interface::CallbackReturn PendulumPID::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PendulumPID::command_interface_configuration() const
    {
        // export command interfaces for state estimation
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interface_config.names.push_back(
            std::string("pivot/")+"effort");
        return command_interface_config; // command_interfaces_    
    }

    controller_interface::InterfaceConfiguration PendulumPID::state_interface_configuration() const
    {
        // use no state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn PendulumPID::on_configure(const rclcpp_lifecycle::State &)
    {
        kp_ = auto_declare<double>("kp", 0.0);
        kd_ = auto_declare<double>("kd", 0.0);
        ki_ = auto_declare<double>("ki", 0.0);
        imax_ = auto_declare<double>("imax", 1e6);
        imin_ = auto_declare<double>("imin", -1e6);
        pid_controller_.init(1); // single joint
        pid_controller_.set_gains(0, kp_, ki_, kd_, imax_, imin_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumPID::on_cleanup(const rclcpp_lifecycle::State &)
    {
        pid_controller_.cleanup();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumPID::on_activate(const rclcpp_lifecycle::State &)
    {
        pid_controller_.reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumPID::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> PendulumPID::on_export_reference_interfaces()
    {
        if (reference_interfaces_.empty()) {
            // We need to create storage for all references: joints (3) + tip (4)
            // Total: 3 + 4 = 7 values
            reference_interfaces_.resize(7, 0.0);
        }
        std::string controller_name = this->get_node()->get_name();
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_position", &reference_interfaces_[0]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_velocity", &reference_interfaces_[1]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_effort", &reference_interfaces_[2]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_y", &reference_interfaces_[3]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_z", &reference_interfaces_[4]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_vy", &reference_interfaces_[5]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_vz", &reference_interfaces_[6]));
        return reference_interfaces;
    }

    bool PendulumPID::on_set_chained_mode(bool chained_mode)
    {
        return true; // enable chaining since this is a controller and leverage command interface
    }

    controller_interface::return_type PendulumPID::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // get the current state
        double position = reference_interfaces_[0]; // joint_position
        double velocity = reference_interfaces_[1]; // joint_velocity

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type PendulumPID::update_reference_from_subscribers()
    {
        return controller_interface::return_type::OK;
    }

    
}; // namespace pendulum_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pendulum_controllers::PendulumPID, controller_interface::ChainableControllerInterface);