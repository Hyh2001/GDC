#include "base_quadruped_estimators/base_quadruped_estimator.hpp"

namespace base_quadruped_estimators
{ 
    controller_interface::CallbackReturn BaseQuadrupedEstimator::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }   

    controller_interface::InterfaceConfiguration BaseQuadrupedEstimator::command_interface_configuration() const
    {
        // use no command interfaces
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interface_config; // command_interfaces_
    }

    controller_interface::InterfaceConfiguration BaseQuadrupedEstimator::state_interface_configuration() const
    {
        // use all state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::ALL;
        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn BaseQuadrupedEstimator::on_configure(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BaseQuadrupedEstimator::on_activate(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BaseQuadrupedEstimator::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool BaseQuadrupedEstimator::on_set_chained_mode(bool chained_mode)
    {
        return true; // disable chaining since this is an estimator and leverage state interface
    }

    controller_interface::return_type BaseQuadrupedEstimator::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        size_t idx = 0;
        
        // Joint interfaces - same order as in on_export_reference_interfaces()
        for (size_t j = 0; j < joint_names.size(); ++j) {
            for (size_t i = 0; i < joint_interface_types.size(); ++i) {
                const auto &iface = joint_interface_types[i];
                if (iface == "position") {
                    reference_interfaces_[idx] = joint_pos_[j];
                } else if (iface == "velocity") {
                    reference_interfaces_[idx] = joint_vel_[j];
                } else if (iface == "acceleration") {
                    reference_interfaces_[idx] = joint_acc_[j];
                } else if (iface == "effort") {
                    reference_interfaces_[idx] = joint_tau_[j];
                }
                idx++;
            }
        }
        
        // Contact sensors - SAME ORDER as in on_export_reference_interfaces()
        for (size_t f = 0; f < foot_names.size(); ++f) {
            for (size_t s = 0; s < foot_sensor_names.size(); ++s) {
                const auto &sensor = foot_sensor_names[s];
                if (sensor == "state") {
                    reference_interfaces_[idx] = contact_states_[f];
                } else if (sensor == "force_x") {
                    reference_interfaces_[idx] = contact_forces_[f * 3];
                } else if (sensor == "force_y") {
                    reference_interfaces_[idx] = contact_forces_[f * 3 + 1];
                } else if (sensor == "force_z") {
                    reference_interfaces_[idx] = contact_forces_[f * 3 + 2];
                }
                idx++;
            }
        }
        
        // Position - SAME ORDER as in on_export_reference_interfaces()
        reference_interfaces_[idx++] = pos_[0]; // x
        reference_interfaces_[idx++] = pos_[1]; // y
        reference_interfaces_[idx++] = pos_[2]; // z
        
        // Orientation - SAME ORDER as in on_export_reference_interfaces()
        reference_interfaces_[idx++] = orientation_[0]; // w
        reference_interfaces_[idx++] = orientation_[1]; // x
        reference_interfaces_[idx++] = orientation_[2]; // y
        reference_interfaces_[idx++] = orientation_[3]; // z
        
        // Linear velocity - SAME ORDER as in on_export_reference_interfaces()
        reference_interfaces_[idx++] = lin_vel_[0]; // x
        reference_interfaces_[idx++] = lin_vel_[1]; // y
        reference_interfaces_[idx++] = lin_vel_[2]; // z
        
        // Angular velocity - SAME ORDER as in on_export_reference_interfaces()
        reference_interfaces_[idx++] = ang_vel_[0]; // x
        reference_interfaces_[idx++] = ang_vel_[1]; // y
        reference_interfaces_[idx++] = ang_vel_[2]; // z
        
        // Linear acceleration - SAME ORDER as in on_export_reference_interfaces()
        reference_interfaces_[idx++] = lin_acc_[0]; // x
        reference_interfaces_[idx++] = lin_acc_[1]; // y
        reference_interfaces_[idx++] = lin_acc_[2]; // z
        
        // Angular acceleration - SAME ORDER as in on_export_reference_interfaces()
        reference_interfaces_[idx++] = ang_acc_[0]; // x
        reference_interfaces_[idx++] = ang_acc_[1]; // y
        reference_interfaces_[idx++] = ang_acc_[2]; // z

        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> BaseQuadrupedEstimator::on_export_reference_interfaces()
    {
        // Initialize storage for reference values if not already done in header
        if (reference_interfaces_.empty()) {
            // We need to create storage for all references: joints (12*4) + contacts (4*4) + pos (3) + ori (4) + velocities (6) + accelerations (6)
            // Total: 48 + 16 + 19 = 83 values
            reference_interfaces_.resize(83, 0.0);
        }
        std::vector<hardware_interface::CommandInterface> interfaces;
        std::string controller_name = this->get_node()->get_name();
        size_t idx = 0;
        
        // Joint interfaces: "<joint>/<interface>"
        for (const auto &joint : joint_names) {
            for (const auto &iface : joint_interface_types) {
                interfaces.emplace_back(controller_name, 
                                    joint + "/" + iface, 
                                    &reference_interfaces_[idx++]);
            }
        }
        
        // Contact sensors: "<foot>/state", "<foot>/force_x", "<foot>/force_y", "<foot>/force_z"
        for (const auto &foot : foot_names) {
            for (const auto &sensor : foot_sensor_names) {
                interfaces.emplace_back(controller_name,
                                    foot + "/" + sensor,
                                    &reference_interfaces_[idx++]);
            }
        }
        
        // Position
        interfaces.emplace_back(controller_name, pos_name + "/x", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, pos_name + "/y", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, pos_name + "/z", &reference_interfaces_[idx++]);
        
        // Orientation
        interfaces.emplace_back(controller_name, ori_name + "/w", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, ori_name + "/x", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, ori_name + "/y", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, ori_name + "/z", &reference_interfaces_[idx++]);
        
        // Linear velocity
        interfaces.emplace_back(controller_name, lin_vel_name + "/x", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, lin_vel_name + "/y", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, lin_vel_name + "/z", &reference_interfaces_[idx++]);
        
        // Angular velocity
        interfaces.emplace_back(controller_name, ang_vel_name + "/x", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, ang_vel_name + "/y", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, ang_vel_name + "/z", &reference_interfaces_[idx++]);
        
        // Linear acceleration
        interfaces.emplace_back(controller_name, lin_acc_name + "/x", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, lin_acc_name + "/y", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, lin_acc_name + "/z", &reference_interfaces_[idx++]);
        
        // Angular acceleration
        interfaces.emplace_back(controller_name, ang_acc_name + "/x", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, ang_acc_name + "/y", &reference_interfaces_[idx++]);
        interfaces.emplace_back(controller_name, ang_acc_name + "/z", &reference_interfaces_[idx++]);
        
        return interfaces;
    }

    controller_interface::return_type BaseQuadrupedEstimator::update_reference_from_subscribers()
    {
        return controller_interface::return_type::OK;
    }

} // namespace base_quadruped_estimators

