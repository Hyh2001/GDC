#include "base_quadruped_estimators/dummy_estimator.hpp"

namespace quadruped_controllers
{

    controller_interface::CallbackReturn DummyEstimator::on_init()
    {
        // create the node and corresponding subscriber
        node_ptr_ = rclcpp::Node::make_shared(node_name_);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

        QuadEst_subscriber_ = node_ptr_->create_subscription<quadruped_msgs::msg::QuadEst>(
            subscribe_topic_name_, qos,
            [this](const quadruped_msgs::msg::QuadEst::SharedPtr msg)
            {
                for (int i = 0; i < 12; i++)
                {
                    joint_pos_[i] = msg->motor_state[i].q;
                    joint_vel_[i] = msg->motor_state[i].dq;
                    joint_acc_[i] = msg->motor_state[i].ddq;
                    joint_tau_[i] = msg->motor_state[i].tau;
                }
                for (int i = 0; i < 4; i++)
                {
                    contact_states_[i] = msg->contact_state[i].contact;
                    contact_forces_[i * 3] = msg->contact_force[i].force.x; // assuming
                    contact_forces_[i * 3 + 1] = msg->contact_force[i].force.y;
                    contact_forces_[i * 3 + 2] = msg->contact_force[i].force.z;
                }
                pos_[0] = msg->pose.position.x;
                pos_[1] = msg->pose.position.y;
                pos_[2] = msg->pose.position.z;
                orientation_[0] = msg->pose.orientation.w;
                orientation_[1] = msg->pose.orientation.x;
                orientation_[2] = msg->pose.orientation.y;
                orientation_[3] = msg->pose.orientation.z;
                lin_vel_[0] = msg->twist.linear.x;
                lin_vel_[1] = msg->twist.linear.y;
                lin_vel_[2] = msg->twist.linear.z;
                ang_vel_[0] = msg->twist.angular.x;
                ang_vel_[1] = msg->twist.angular.y;
                ang_vel_[2] = msg->twist.angular.z;
                lin_acc_[0] = msg->accel.linear.x;
                lin_acc_[1] = msg->accel.linear.y;
                lin_acc_[2] = msg->accel.linear.z;
                ang_acc_[0] = msg->accel.angular.x;
                ang_acc_[1] = msg->accel.angular.y;
                ang_acc_[2] = msg->accel.angular.z;
            });

        executor_.add_node(node_ptr_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DummyEstimator::on_configure(const rclcpp_lifecycle::State &)
    {
    }

    controller_interface::CallbackReturn DummyEstimator::on_activate(const rclcpp_lifecycle::State &)
    {
    }

    controller_interface::CallbackReturn DummyEstimator::on_deactivate(const rclcpp_lifecycle::State &)
    {
    }

    controller_interface::InterfaceConfiguration DummyEstimator::command_interface_configuration() const
    {
        // no command interfaces
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        // joint interfaces: "<joint>/<interface>"
        for (const auto & joint : joint_names) {
            for (const auto & iface : joint_interface_types) {
                command_interface_config.names.push_back(joint + "/" + iface);
            }
        }
        // contact sensors "<foot>/state", "<foot>/force_x", "<foot>/force_y", "<foot>/force_z"
        for (const auto & foot : foot_names) {
            for (const auto & sensor : foot_sensor_names) {
                command_interface_config.names.push_back(foot + "/" + sensor);
            }
        }
        // 
        
        
        return command_interface_config;
    }

    controller_interface::InterfaceConfiguration DummyEstimator::state_interface_configuration() const
    {
        // use all the state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::ALL;
        return state_interface_config;
    }

}; // namespace quadruped_controllers