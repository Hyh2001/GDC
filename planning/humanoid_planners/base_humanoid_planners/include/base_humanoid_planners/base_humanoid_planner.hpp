#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

namespace humanoid_planners
{ 
    class BaseHumanoidPlanner : public controller_interface::ChainableControllerInterface
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

    protected:
        std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        // sensor reading fields
        std::vector<std::string> joint_names_ = {};
        int num_joints_ = 0;
        std::vector<std::string> joint_state_interface_types_ = {"position", "velocity", "effort"};
        std::vector<std::string> joint_command_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
        std::vector<std::string> foot_names_ = {};
        std::vector<std::string> foot_sensor_names_ = {};
        std::string pos_name_ = "global_pos";
        std::string ori_name_ = "orientation"; // wxyz
        std::string lin_vel_name_ = "global_lin_vel";
        std::string ang_vel_name_ = "global_ang_vel";
        std::string lin_acc_name_ = "global_lin_acc";
        std::string ang_acc_name_ = "global_ang_acc";
        // readings L -> R
        std::vector<double> joint_pos_ = {};
        std::vector<double> joint_vel_ = {};
        std::vector<double> joint_acc_ = {};
        std::vector<double> joint_tau_ = {};
        std::array<double, 3> pos_ = {0.0, 0.0, 0.0};
        std::array<double, 4> ori_ = {1.0, 0.0, 0.0, 0.0}; // wxyz
        std::array<double, 3> lin_vel_ = {0.0, 0.0, 0.0};
        std::array<double, 3> ang_vel_ = {0.0, 0.0, 0.0};
        std::array<double, 3> lin_acc_ = {0.0, 0.0, 0.0};
        std::array<double, 3> ang_acc_ = {0.0, 0.0, 0.0};
        std::array<double, 2> contact_states_ = {false, false}; // left, right
        std::array<std::array<double, 6>, 2> contact_wrenches_ = {{
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // left
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}   // right
        }};
        // ros2 related
        std::string estimator_name_ = "";
        std::string ref_planner_name_ = "";
    };



}; // namespace humanoid_planners