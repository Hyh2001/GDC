#include "pendulum_estimators/dummy_estimator.hpp"

namespace pendulum_estimators
{

    controller_interface::CallbackReturn DummyEstimator::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DummyEstimator::command_interface_configuration() const
    {
        // export command interfaces for state estimation
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interface_config; // command_interfaces_
    }

    controller_interface::InterfaceConfiguration DummyEstimator::state_interface_configuration() const
    {
        // use no state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interface_config; // state_interfaces_
    }

    std::vector<hardware_interface::StateInterface> DummyEstimator::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        std::string estimator_name = this->get_name();
        state_interfaces.emplace_back(
                estimator_name, "joint_position_est", &joint_pos_);
        state_interfaces.emplace_back(
                estimator_name, "joint_velocity_est", &joint_vel_);
        state_interfaces.emplace_back(
                estimator_name, "joint_torque_est", &joint_tau_);
        state_interfaces.emplace_back(
                estimator_name, "tip_position_y_est", &tip_pos_[0]);
        state_interfaces.emplace_back(
                estimator_name, "tip_position_z_est", &tip_pos_[1]);
        state_interfaces.emplace_back(
                estimator_name, "tip_velocity_y_est", &tip_vel_[0]);
        state_interfaces.emplace_back(
                estimator_name, "tip_velocity_z_est", &tip_vel_[1]);
        return state_interfaces;

    }

    controller_interface::CallbackReturn DummyEstimator::on_configure(const rclcpp_lifecycle::State &)
    {
        // load node_name_ and subscribe_topic_name_ parameters
        node_name_ = auto_declare<std::string>("node_name", "");
        subscribe_topic_name_ = auto_declare<std::string>("subscribe_topic_name", "");

        if(node_name_.empty() || subscribe_topic_name_.empty())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(),
                "DummyEstimator: 'node_name' or 'subscribe_topic_name' parameter is empty.");
            return controller_interface::CallbackReturn::FAILURE;
        }

        // create the node and corresponding subscriber
        node_ptr_ = rclcpp::Node::make_shared(node_name_);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

        PendulumEst_subscriber_ = node_ptr_->create_subscription<pend_msgs::msg::PendulumEst>(
            subscribe_topic_name_, qos,
            [this](const pend_msgs::msg::PendulumEst::SharedPtr msg)
            {
                joint_pos_ = msg->motor_state.q;
                joint_vel_ = msg->motor_state.dq;
                joint_tau_ = msg->motor_state.tau;

                tip_pos_[0] = msg->tip_state[0]; // y
                tip_pos_[1] = msg->tip_state[1]; // z
                tip_vel_[0] = msg->tip_state[2]; // vy
                tip_vel_[1] = msg->tip_state[3]; // vz
            });

        executor_.add_node(node_ptr_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DummyEstimator::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        executor_.spin_some(std::chrono::milliseconds(0));  

        // command_interfaces_[0].set_value(joint_pos_);
        // command_interfaces_[1].set_value(joint_vel_);
        // command_interfaces_[2].set_value(joint_tau_);
        // command_interfaces_[3].set_value(tip_pos_[0]); // y
        // command_interfaces_[4].set_value(tip_pos_[1]); // z
        // command_interfaces_[5].set_value(tip_vel_[0]); // vy
        // command_interfaces_[6].set_value(tip_vel_[1]); // vz
        // std::cout << "DummyEstimator: pos=" << joint_pos_ << ", vel=" << joint_vel_ << std::endl;
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type DummyEstimator::update_reference_from_subscribers(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Nothing to do here since all updates are handled in update_and_write_commands
        return controller_interface::return_type::OK;
    }

}; // namespace pendulum_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pendulum_estimators::DummyEstimator, controller_interface::ChainableControllerInterface);