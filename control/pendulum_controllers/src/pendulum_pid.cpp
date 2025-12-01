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
        // use state interfaces exported from estimators and planners
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // state estimation
        state_interface_config.names.push_back(estimator_name_ + "/joint_position_est");
        state_interface_config.names.push_back(estimator_name_ + "/joint_velocity_est");
        state_interface_config.names.push_back(estimator_name_ + "/joint_torque_est");
        state_interface_config.names.push_back(estimator_name_ + "/tip_position_y_est");
        state_interface_config.names.push_back(estimator_name_ + "/tip_position_z_est");
        state_interface_config.names.push_back(estimator_name_ + "/tip_velocity_y_est");
        state_interface_config.names.push_back(estimator_name_ + "/tip_velocity_z_est");

        // reference trajectory
        state_interface_config.names.push_back(planner_name_ + "/joint_position_ref");
        state_interface_config.names.push_back(planner_name_ + "/joint_velocity_ref");

        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn PendulumPID::on_configure(const rclcpp_lifecycle::State &)
    {
        estimator_name_ = auto_declare<std::string>("estimator_name", "estimator");
        planner_name_ = auto_declare<std::string>("planner_name", "planner");
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

    controller_interface::return_type PendulumPID::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // get the current state
        double position = state_interfaces_[0].get_value(); // joint_position
        double velocity = state_interfaces_[1].get_value(); // joint_velocity
        double position_ref = state_interfaces_[7].get_value(); // joint_position_ref
        double velocity_ref = state_interfaces_[8].get_value(); // joint_velocity_ref
        // std::cout << "PendulumPID: pos_ref=" << position_ref << ", pos=" << position << std::endl;
        double error = position_ref - position;
        double error_dot = velocity_ref - velocity;
        // compute control effort
        double effort_command = pid_controller_.compute(
            0, error, period);
        // write to command interface
        command_interfaces_[0].set_value(effort_command); // effort

        return controller_interface::return_type::OK;
    }
    
}; // namespace pendulum_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pendulum_controllers::PendulumPID, controller_interface::ControllerInterface);