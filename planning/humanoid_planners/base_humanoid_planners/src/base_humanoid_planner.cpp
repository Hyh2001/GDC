#include "base_humanoid_planners/base_humanoid_planner.hpp"

namespace humanoid_planners
{

    controller_interface::CallbackReturn BaseHumanoidPlanner::on_init()
    {
        // estimator name
        estimator_name_ = auto_declare<std::string>("estimator_name", estimator_name_);
        ref_planner_name_ = auto_declare<std::string>(
            "ref_planner_name", ref_planner_name_);

        // interface names for parameters
        joint_names_ = auto_declare<std::vector<std::string>>("joint_names", joint_names_);
        num_joints_ = joint_names_.size();
        joint_state_interface_types_ = auto_declare<std::vector<std::string>>(
            "joint_state_interfaces", joint_state_interface_types_);
        joint_command_interface_types_ = auto_declare<std::vector<std::string>>(
            "joint_command_interfaces", joint_command_interface_types_);
        foot_names_ = auto_declare<std::vector<std::string>>(
            "foot_names", foot_names_);
        foot_sensor_names_ = auto_declare<std::vector<std::string>>(
            "foot_sensor_names", foot_sensor_names_);
        pos_name_ = auto_declare<std::string>("pos_name", pos_name_);
        ori_name_ =  auto_declare<std::string>("ori_name", ori_name_);
        lin_vel_name_ = auto_declare<std::string>("lin_vel_name", lin_vel_name_);
        ang_vel_name_ = auto_declare<std::string>("ang_vel_name", ang_vel_name_);
        lin_acc_name_ = auto_declare<std::string>("lin_acc_name", lin_acc_name_);
        ang_acc_name_ = auto_declare<std::string>("ang_acc_name", ang_acc_name_);

        joint_pos_.resize(num_joints_, 0.0);
        joint_vel_.resize(num_joints_, 0.0);
        joint_acc_.resize(num_joints_, 0.0);
        joint_tau_.resize(num_joints_, 0.0);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration BaseHumanoidPlanner::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }

    controller_interface::InterfaceConfiguration BaseHumanoidPlanner::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }

    controller_interface::CallbackReturn BaseHumanoidPlanner::on_configure(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BaseHumanoidPlanner::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BaseHumanoidPlanner::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> BaseHumanoidPlanner::on_export_state_interfaces()
    {
        return {};
    }

    controller_interface::return_type BaseHumanoidPlanner::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return controller_interface::return_type::OK;
    }
};
