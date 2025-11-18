#include "pendulum_estimators/base_pendulum_estimator.hpp"

namespace pendulum_estimators
{ 
    controller_interface::CallbackReturn BasePendulumEstimator::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }   

    controller_interface::InterfaceConfiguration BasePendulumEstimator::command_interface_configuration() const
    {
        // use no command interfaces
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interface_config; // command_interfaces_
    }

    controller_interface::InterfaceConfiguration BasePendulumEstimator::state_interface_configuration() const
    {
        // use all state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::ALL;
        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn BasePendulumEstimator::on_configure(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasePendulumEstimator::on_activate(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasePendulumEstimator::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool BasePendulumEstimator::on_set_chained_mode(bool chained_mode)
    {
        return true; // disable chaining since this is an estimator and leverage state interface
    }

    controller_interface::return_type BasePendulumEstimator::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        size_t idx = 0;
        
        // Joint interfaces - same order as in on_export_reference_interfaces()
        for (size_t i=0; i < joint_interface_types.size(); ++i) {
            const auto &iface = joint_interface_types[i];
            if (iface == "position") {
                reference_interfaces_[idx] = joint_pos_;
            } else if (iface == "velocity") {
                reference_interfaces_[idx] = joint_vel_;
            } else if (iface == "effort") {
                reference_interfaces_[idx] = joint_tau_;
            }
            idx++;
        }
        
        // Tip interfaces - same order as in on_export_reference_interfaces()
        for (size_t i=0; i < tip_interface_types.size(); ++i) {
            const auto &iface = tip_interface_types[i];
            if (iface == "y") {
                reference_interfaces_[idx] = tip_pos_[0];
            } else if (iface == "z") {
                reference_interfaces_[idx] = tip_pos_[1];
            } else if (iface == "vy") {
                reference_interfaces_[idx] = tip_vel_[0];
            } else if (iface == "vz") {
                reference_interfaces_[idx] = tip_vel_[1];
            }
            idx++;
        }

        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> BasePendulumEstimator::on_export_reference_interfaces()
    {
        // Initialize storage for reference values if not already done in header
        if (reference_interfaces_.empty()) {
            // We need to create storage for all references: joints (1*3) + tip (1*4)
            // Total: 3 + 4 = 7 values
            reference_interfaces_.resize(7, 0.0);
        }
        std::vector<hardware_interface::CommandInterface> interfaces;
        std::string controller_name = this->get_node()->get_name();
        size_t idx = 0;
        
        // Joint interfaces: "<joint>/<interface>"
        for (const auto &iface : joint_interface_types) {
            interfaces.emplace_back(controller_name, 
                                joint_name + "/" + iface, 
                                &reference_interfaces_[idx++]);
        }
        
        // Tip interfaces: "<tip>/<interface>"
        for (const auto &iface : tip_interface_types) {
            interfaces.emplace_back(controller_name, 
                                tip_name + "/" + iface, 
                                &reference_interfaces_[idx++]);
        }
        
        return interfaces;
    }

    controller_interface::return_type BasePendulumEstimator::update_reference_from_subscribers()
    {
        return controller_interface::return_type::OK;
    }

} // namespace pendulum_estimators

