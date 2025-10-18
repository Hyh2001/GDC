#include "base_quadruped_estimators/base_quadruped_estimator.hpp"

namespace quadruped_controllers
{ 
    controller_interface::CallbackReturn BaseQuadrupedEstimator::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }   

    controller_interface::InterfaceConfiguration BaseQuadrupedEstimator::command_interface_configuration() const
    {
        // no command interfaces
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // joint interfaces: "<joint>/<interface>"
        for (const auto &joint : joint_names)
        {
            for (const auto &iface : joint_interface_types)
            {
                command_interface_config.names.push_back(joint + "/" + iface);
            }
        }
        // contact sensors "<foot>/state", "<foot>/force_x", "<foot>/force_y", "<foot>/force_z"
        for (const auto &foot : foot_names)
        {
            for (const auto &sensor : foot_sensor_names)
            {
                command_interface_config.names.push_back(foot + "/" + sensor);
            }
        }
        // position
        command_interface_config.names.push_back(pos_name + "/x");
        command_interface_config.names.push_back(pos_name + "/y");
        command_interface_config.names.push_back(pos_name + "/z");
        // orientation
        command_interface_config.names.push_back(ori_name + "/w");
        command_interface_config.names.push_back(ori_name + "/x");
        command_interface_config.names.push_back(ori_name + "/y");
        command_interface_config.names.push_back(ori_name + "/z");
        // linear velocity
        command_interface_config.names.push_back(lin_vel_name + "/x");
        command_interface_config.names.push_back(lin_vel_name + "/y");
        command_interface_config.names.push_back(lin_vel_name + "/z");
        // angular velocity
        command_interface_config.names.push_back(ang_vel_name + "/x");
        command_interface_config.names.push_back(ang_vel_name + "/y");
        command_interface_config.names.push_back(ang_vel_name + "/z");
        // linear acceleration
        command_interface_config.names.push_back(lin_acc_name + "/x");
        command_interface_config.names.push_back(lin_acc_name + "/y");
        command_interface_config.names.push_back(lin_acc_name + "/z");
        // angular acceleration
        command_interface_config.names.push_back(ang_acc_name + "/x");
        command_interface_config.names.push_back(ang_acc_name + "/y");
        command_interface_config.names.push_back(ang_acc_name + "/z");

        return command_interface_config; // command_interfaces_
    }

    controller_interface::InterfaceConfiguration BaseQuadrupedEstimator::state_interface_configuration() const
    {
        // use no state interfaces
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
        return true; // always training since this is an estimator
    }

    controller_interface::return_type BaseQuadrupedEstimator::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Write to command interfaces in the same order as we declared them
        size_t idx = 0;

        // Joint interfaces: "<joint>/<interface>"
        for (size_t j = 0; j < joint_names.size(); ++j)
        {
            for (const auto &iface : joint_interface_types)
            {
                if (idx >= command_interfaces_.size())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds: %zu", idx);
                    return controller_interface::return_type::ERROR;
                }

                if (iface == "position")
                {
                    command_interfaces_[idx].set_value(joint_pos_[j]);
                }
                else if (iface == "velocity")
                {
                    command_interfaces_[idx].set_value(joint_vel_[j]);
                }
                else if (iface == "acceleration")
                {
                    command_interfaces_[idx].set_value(joint_acc_[j]);
                }
                else if (iface == "effort")
                {
                    command_interfaces_[idx].set_value(joint_tau_[j]);
                }
                idx++;
            }
        }

        // Contact sensors "<foot>/state", "<foot>/force_x", "<foot>/force_y", "<foot>/force_z"
        for (size_t f = 0; f < foot_names.size(); ++f)
        {
            for (size_t s = 0; s < foot_sensor_names.size(); ++s)
            {
                if (idx >= command_interfaces_.size())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds: %zu", idx);
                    return controller_interface::return_type::ERROR;
                }

                if (foot_sensor_names[s] == "state")
                {
                    command_interfaces_[idx].set_value(contact_states_[f]);
                }
                else if (foot_sensor_names[s] == "force_x")
                {
                    command_interfaces_[idx].set_value(contact_forces_[f * 3]);
                }
                else if (foot_sensor_names[s] == "force_y")
                {
                    command_interfaces_[idx].set_value(contact_forces_[f * 3 + 1]);
                }
                else if (foot_sensor_names[s] == "force_z")
                {
                    command_interfaces_[idx].set_value(contact_forces_[f * 3 + 2]);
                }
                idx++;
            }
        }

        // Position
        if (idx + 2 < command_interfaces_.size())
        {
            command_interfaces_[idx++].set_value(pos_[0]); // x
            command_interfaces_[idx++].set_value(pos_[1]); // y
            command_interfaces_[idx++].set_value(pos_[2]); // z
        }
        else
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds for position");
            return controller_interface::return_type::ERROR;
        }

        // Orientation
        if (idx + 3 < command_interfaces_.size())
        {
            command_interfaces_[idx++].set_value(orientation_[0]); // w
            command_interfaces_[idx++].set_value(orientation_[1]); // x
            command_interfaces_[idx++].set_value(orientation_[2]); // y
            command_interfaces_[idx++].set_value(orientation_[3]); // z
        }
        else
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds for orientation");
            return controller_interface::return_type::ERROR;
        }

        // Linear velocity
        if (idx + 2 < command_interfaces_.size())
        {
            command_interfaces_[idx++].set_value(lin_vel_[0]); // x
            command_interfaces_[idx++].set_value(lin_vel_[1]); // y
            command_interfaces_[idx++].set_value(lin_vel_[2]); // z
        }
        else
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds for linear velocity");
            return controller_interface::return_type::ERROR;
        }

        // Angular velocity
        if (idx + 2 < command_interfaces_.size())
        {
            command_interfaces_[idx++].set_value(ang_vel_[0]); // x
            command_interfaces_[idx++].set_value(ang_vel_[1]); // y
            command_interfaces_[idx++].set_value(ang_vel_[2]); // z
        }
        else
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds for angular velocity");
            return controller_interface::return_type::ERROR;
        }

        // Linear acceleration
        if (idx + 2 < command_interfaces_.size())
        {
            command_interfaces_[idx++].set_value(lin_acc_[0]); // x
            command_interfaces_[idx++].set_value(lin_acc_[1]); // y
            command_interfaces_[idx++].set_value(lin_acc_[2]); // z
        }
        else
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds for linear acceleration");
            return controller_interface::return_type::ERROR;
        }

        // Angular acceleration
        if (idx + 2 < command_interfaces_.size())
        {
            command_interfaces_[idx++].set_value(ang_acc_[0]); // x
            command_interfaces_[idx++].set_value(ang_acc_[1]); // y
            command_interfaces_[idx++].set_value(ang_acc_[2]); // z
        }
        else
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Command interface index out of bounds for angular acceleration");
            return controller_interface::return_type::ERROR;
        }

        return controller_interface::return_type::OK;
    }

} // namespace quadruped_controllers

