#include "base_humanoid_estimators/humanoid_dummy_estimator.hpp"

namespace humanoid_estimators
{
    controller_interface::CallbackReturn HumanoidDummyEstimator::on_init()
    {
        BaseHumanoidEstimator::on_init();
        
        // load node_name_ and subscribe_topic_name_ parameters
        node_name_ = auto_declare<std::string>("node_name", "");
        subscribe_topic_name_ = auto_declare<std::string>("subscribe_topic_name", "");

        if(node_name_.empty() || subscribe_topic_name_.empty())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(),
                "HumanoidDummyEstimator: 'node_name' or 'subscribe_topic_name' parameter is empty.");
            return controller_interface::CallbackReturn::FAILURE;
        }

        // create the node and corresponding subscriber
        node_ptr_ = rclcpp::Node::make_shared(node_name_);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

        HumanoidEst_subscriber_ = node_ptr_->create_subscription<humanoid_msgs::msg::HumanoidEst>(
            subscribe_topic_name_, qos,
            [this](const humanoid_msgs::msg::HumanoidEst::SharedPtr msg)
            {
                if (joint_pos_.size() != msg->motor_state.size())
                {
                    joint_pos_.resize(msg->motor_state.size(), 0.0);
                    joint_vel_.resize(msg->motor_state.size(), 0.0);
                    joint_acc_.resize(msg->motor_state.size(), 0.0);
                    joint_tau_.resize(msg->motor_state.size(), 0.0);
                }
                for (size_t i = 0; i < joint_pos_.size() ; i++)
                {
                    joint_pos_[i] = msg->motor_state[i].q;
                    joint_vel_[i] = msg->motor_state[i].dq;
                    joint_acc_[i] = msg->motor_state[i].ddq;
                    joint_tau_[i] = msg->motor_state[i].tau;
                }
                for (int i = 0; i < 2; i++)
                {
                    contact_states_[i] = msg->contact_state[i].contact;
                    contact_wrenches_[i][0] = msg->contact_wrench[i].wrench.force.x; 
                    contact_wrenches_[i][1] = msg->contact_wrench[i].wrench.force.y;
                    contact_wrenches_[i][2] = msg->contact_wrench[i].wrench.force.z;
                    contact_wrenches_[i][3] = msg->contact_wrench[i].wrench.torque.x;
                    contact_wrenches_[i][4] = msg->contact_wrench[i].wrench.torque.y;
                    contact_wrenches_[i][5] = msg->contact_wrench[i].wrench.torque.z; 
                }
                pos_[0] = msg->pose.position.x;
                pos_[1] = msg->pose.position.y;
                pos_[2] = msg->pose.position.z;
                ori_[0] = msg->pose.orientation.w;
                ori_[1] = msg->pose.orientation.x;
                ori_[2] = msg->pose.orientation.y;
                ori_[3] = msg->pose.orientation.z;
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

    controller_interface::InterfaceConfiguration HumanoidDummyEstimator::state_interface_configuration() const
    {
        // use no state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn HumanoidDummyEstimator::on_configure(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type HumanoidDummyEstimator::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        executor_.spin_some(std::chrono::milliseconds(1));

        return BaseHumanoidEstimator::update_and_write_commands(time, period);
    }

    controller_interface::return_type HumanoidDummyEstimator::update_reference_from_subscribers(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return controller_interface::return_type::OK;
    }
    
}; // namespace base_humanoid_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(humanoid_estimators::HumanoidDummyEstimator, controller_interface::ChainableControllerInterface);