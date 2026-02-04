#pragma once

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/introspection.hpp"

namespace humanoid_controllers
{

    class BaseHumanoidController : public controller_interface::ChainableControllerInterface
    {
    public:
        controller_interface::CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    protected:
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
            
        controller_interface::return_type update_reference_from_subscribers(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        // void get_state_interface_value(
        //     const std::string & interface_name, double & value
        // ); 

        // void set_command_interface_value(
        //     const std::string & interface_name, const double & value
        // );

        // void get_state_interface_values(
        //     const std::vector<std::string> & interface_names, 
        //     std::vector<double> & values
        // );

        // void set_command_interface_values(
        //     const std::vector<std::string> & interface_names, 
        //     const std::vector<double> & values
        // );

        // predecessors
        std::string estimator_name_ = "";
        std::string planner_name_ = "";
        std::string ref_controller_name_ = "";
        
        // L -> R
        std::vector<std::string> joint_names_ = {};
        // state and command interfaces        
        std::vector<std::string> joint_state_interface_types_ = {"position", "velocity", "effort"};
        std::vector<std::string> joint_command_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
        std::vector<std::string> foot_names_ = {
            "L_foot", "R_foot"
        };
        std::vector<std::string> foot_sensor_names_ = {
            "state", "force_x", "force_y", "force_z", 
            "torque_x", "torque_y", "torque_z" 
        };
        std::string pos_name_ = "global_pos";
        std::string ori_name_ = "orientation"; // wxyz
        // velocity and acceleration interfaces can be either global or linear
        std::string lin_vel_name_ = "global_lin_vel"; 
        std::string ang_vel_name_ = "global_ang_vel";
        std::string lin_acc_name_ = "global_lin_acc";
        std::string ang_acc_name_ = "global_ang_acc";

        // debug
        bool debug_ = false; 
    };
};