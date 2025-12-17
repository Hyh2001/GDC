#include "base_quadruped_estimators/base_quadruped_estimator.hpp"

namespace quadruped_estimators
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

    std::vector<hardware_interface::StateInterface> BaseQuadrupedEstimator::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        std::string estimator_name = this->get_name(); // 
        size_t joint_idx = 0;
        // Joint interfaces 
        for (const auto &joint : joint_names) {
            for (const auto &iface : joint_interface_types) {
                if (iface == "position") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, joint + "_" + iface + "_est", &joint_pos_[joint_idx]));
                } else if (iface == "velocity") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, joint + "_" + iface + "_est", &joint_vel_[joint_idx]));
                } else if (iface == "acceleration") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, joint + "_" + iface + "_est", &joint_acc_[joint_idx]));
                } else if (iface == "effort") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, joint + "_" + iface + "_est", &joint_tau_[joint_idx]));
                }
            }
            joint_idx++;
        }

        size_t foot_idx = 0;
        // Contact sensors and force sensors
        for (const auto &foot : foot_names) {
            for (const auto &sensor : foot_sensor_names) {
                if (sensor == "state") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_states_[foot_idx]));
                } else if (sensor == "force_x") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_forces_[foot_idx * 3]));
                } else if (sensor == "force_y") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_forces_[foot_idx * 3 + 1]));
                } else if (sensor == "force_z") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_forces_[foot_idx * 3 + 2]));
                }
            }
            foot_idx++;
        }

        // Position 
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, pos_name + "_x" + "_est", &pos_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, pos_name + "_y" + "_est", &pos_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, pos_name + "_z" + "_est", &pos_[2]));
    
        // Orientation
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name + "_w" + "_est", &orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name + "_x" + "_est", &orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name + "_y" + "_est", &orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name + "_z" + "_est", &orientation_[3]));
            
        // Global Linear velocity
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_vel_name + "_x" + "_est", &lin_vel_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_vel_name + "_y" + "_est", &lin_vel_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_vel_name + "_z" + "_est", &lin_vel_[2]));

        // Global Angular velocity
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_vel_name + "_x" + "_est", &ang_vel_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_vel_name + "_y" + "_est", &ang_vel_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_vel_name + "_z" + "_est", &ang_vel_[2]));

        // Global Linear acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_acc_name + "_x" + "_est", &lin_acc_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_acc_name + "_y" + "_est", &lin_acc_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_acc_name + "_z" + "_est", &lin_acc_[2]));
        
        // Global Angular acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_acc_name + "_x" + "_est", &ang_acc_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_acc_name + "_y" + "_est", &ang_acc_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_acc_name + "_z" + "_est", &ang_acc_[2]));
    
        return state_interfaces;
    }   

    controller_interface::return_type BaseQuadrupedEstimator::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type BaseQuadrupedEstimator::update_reference_from_subscribers(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return controller_interface::return_type::OK;
    }

} // namespace quadruped_estimators

