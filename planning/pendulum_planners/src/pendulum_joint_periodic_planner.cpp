#include "pendulum_planners/pendulum_joint_periodic_planner.hpp"

namespace pendulum_planners
{

    controller_interface::CallbackReturn PendulumJointPeriodicPlanner::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PendulumJointPeriodicPlanner::command_interface_configuration() const
    {
        // export command interfaces for reference traj
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interface_config; // command_interfaces_
    }

    controller_interface::InterfaceConfiguration PendulumJointPeriodicPlanner::state_interface_configuration() const
    {
        // use no state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn PendulumJointPeriodicPlanner::on_configure(const rclcpp_lifecycle::State &)
    {
        offset_ = auto_declare<double>("offset", 0.0);
        amplitude_ = auto_declare<double>("amplitude", 1.0);
        frequency_ = auto_declare<double>("frequency", 0.5); // Hz
        phase_ = auto_declare<double>("phase", 0.0); // rad
        sinusoid_planner_ptr_ = std::make_shared<control_toolbox::Sinusoid>(
            offset_, amplitude_, frequency_, phase_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumJointPeriodicPlanner::on_cleanup(const rclcpp_lifecycle::State &)
    {
        sinusoid_planner_ptr_ = nullptr;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumJointPeriodicPlanner::on_activate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumJointPeriodicPlanner::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PendulumJointPeriodicPlanner::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        std::string planner_name = this->get_name();
        state_interfaces.emplace_back(
            planner_name, "joint_position_ref", &q_);
        state_interfaces.emplace_back(
            planner_name, "joint_velocity_ref", &qd_);
        return state_interfaces;
    }

    controller_interface::return_type PendulumJointPeriodicPlanner::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        q_ = sinusoid_planner_ptr_->update(
            time.nanoseconds() * 1e-9, qd_, qdd_);

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type PendulumJointPeriodicPlanner::update_reference_from_subscribers(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // no-op since this planner does not subscribe to any topic
        return controller_interface::return_type::OK;
    }


}; // namespace pendulum_planners

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pendulum_planners::PendulumJointPeriodicPlanner, controller_interface::ChainableControllerInterface);
