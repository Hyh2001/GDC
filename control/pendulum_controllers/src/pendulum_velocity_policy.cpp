#include "pendulum_controllers/pendulum_velocity_policy.hpp"

namespace pendulum_controllers
{

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PendulumVelocityPolicy::command_interface_configuration() const
    {
        // export command interfaces for state estimation
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interface_config.names.push_back(
            std::string("pivot/")+"position");
        command_interface_config.names.push_back(
            std::string("pivot/")+"kp");
        command_interface_config.names.push_back(
            std::string("pivot/")+"kd");
        return command_interface_config; // command_interfaces_    
    }

    controller_interface::InterfaceConfiguration PendulumVelocityPolicy::state_interface_configuration() const
    {
        // use no state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_configure(const rclcpp_lifecycle::State &)
    {
        std::string policy_path = auto_declare<std::string>("policy_path", "");
        int input_size = auto_declare<int>("input_size", 3);
        int output_size = auto_declare<int>("output_size", 1);
        kp_ = auto_declare<double>("kp", 0.0);
        kd_ = auto_declare<double>("kd", 0.0);
        velocity_policy_ptr_ = std::make_shared<base_controllers::OnnxPolicy>(policy_path, input_size, output_size);
        input_vector_.resize(input_size, 0.0);
        output_vector_.resize(output_size, 0.0);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_activate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> PendulumVelocityPolicy::on_export_reference_interfaces()
    {
        if (reference_interfaces_.empty()) {
            // We need to create storage for all references: joints (3) + tip (4) + ref(2)
            // Total: 3 + 4 + 2= 9 values
            reference_interfaces_.resize(9, 0.0);
        }
        std::string controller_name = this->get_node()->get_name();
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        // state estimation
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
        // reference
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_position_ref", &reference_interfaces_[7]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_velocity_ref", &reference_interfaces_[8])); 
        return reference_interfaces;
    }

    bool PendulumVelocityPolicy::on_set_chained_mode(bool chained_mode)
    {
        return true; // enable chaining since this is a controller and leverage command interface
    }

    controller_interface::return_type PendulumVelocityPolicy::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // get the current state
        double position = reference_interfaces_[0]; // joint_position
        double velocity = reference_interfaces_[1]; // joint_velocity
        double position_ref = reference_interfaces_[7]; // joint_position_ref
        double velocity_ref = reference_interfaces_[8]; // joint_velocity_ref
        // std::cout << "PendulumPID: pos_ref=" << position_ref << ", pos=" << position << std::endl;
        input_vector_[0] = position;
        input_vector_[1] = velocity; 
        input_vector_[2] = velocity_ref;
        output_vector_ = velocity_policy_ptr_->infer(input_vector_);
        if (output_vector_.size() != 1){
            RCLCPP_ERROR(this->get_node()->get_logger(), "PendulumVelocityPolicy: output size mismatch, expect only one position!");

            return controller_interface::return_type::ERROR;
        }
        // write to command interface
        command_interfaces_[0].set_value(output_vector_[0]); // effort
        command_interfaces_[1].set_value(kp_); // kp
        command_interfaces_[2].set_value(kd_); // kd

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type PendulumVelocityPolicy::update_reference_from_subscribers()
    {
        return controller_interface::return_type::OK;
    }

    
}; // namespace pendulum_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pendulum_controllers::PendulumVelocityPolicy, controller_interface::ChainableControllerInterface);