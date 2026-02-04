#pragma once
#ifndef ROS2_CONTROL_UTILS_HPP
#define ROS2_CONTROL_UTILS_HPP

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

namespace base_utils
{
    inline void get_state_interface_value(
        const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
        const std::string & interface_name, double & value)
    {
        for (const auto &state_iface : state_interfaces) {
            const std::string full_name = state_iface.get_prefix_name() + "/" + state_iface.get_interface_name();
            if (full_name == interface_name) {
                auto opt_value = state_iface.get_optional();
                if (opt_value.has_value()) {
                    value = opt_value.value();
                    return;
                }
            }
        }
        throw std::runtime_error("State interface '" + interface_name + "' not found.");
    }

    inline void set_command_interface_value(
        std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
        const std::string & interface_name, const double & value)
    {
        for (auto &cmd_iface : command_interfaces) {
            const std::string full_name = cmd_iface.get_prefix_name() + "/" + cmd_iface.get_interface_name();
            if (full_name == interface_name) {
                cmd_iface.set_value(value);
                return;
            }
        }
        throw std::runtime_error("Command interface '" + interface_name + "' not found.");
    }

    inline void get_state_interface_values(
        const std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
        const std::vector<std::string> & interface_names,
        std::vector<double> & values)
    {
        if (values.size() != interface_names.size()) {
            values.resize(interface_names.size());
        }
        for (std::size_t i = 0; i < interface_names.size(); ++i) {
            get_state_interface_value(state_interfaces, interface_names[i], values[i]);
        }
    }

    inline void set_command_interface_values(
        std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
        const std::vector<std::string> & interface_names,
        const std::vector<double> & values)
    {
        if (values.size() != interface_names.size()) {
            throw std::runtime_error("Size of values does not match size of interface names.");
        }
        for (std::size_t i = 0; i < interface_names.size(); ++i) {
            set_command_interface_value(command_interfaces, interface_names[i], values[i]);
        }
    }
};

#endif // ROS2_CONTROL_UTILS_HPP