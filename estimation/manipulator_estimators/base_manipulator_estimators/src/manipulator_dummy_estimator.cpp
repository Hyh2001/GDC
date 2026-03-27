#include "base_manipulator_estimators/manipulator_dummy_estimator.hpp"

namespace manipulator_estimators
{
controller_interface::CallbackReturn ManipulatorDummyEstimator::on_init()
{
  BaseManipulatorEstimator::on_init();

  // load node_name_ and subscribe_topic_name_ parameters
  node_name_ = auto_declare<std::string>("node_name", "");
  subscribe_topic_name_ = auto_declare<std::string>("subscribe_topic_name", "");

  if (node_name_.empty() || subscribe_topic_name_.empty())
  {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "ManipulatorDummyEstimator: 'node_name' or 'subscribe_topic_name' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // create the node and corresponding subscriber
  node_ptr_ = rclcpp::Node::make_shared(node_name_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  ManipulatorEst_subscriber_ = node_ptr_->create_subscription<manipulator_msgs::msg::ManipulatorEst>(
      subscribe_topic_name_, qos,
      [this](const manipulator_msgs::msg::ManipulatorEst::SharedPtr msg)
      {
        for (size_t i = 0; i < joint_pos_.size(); i++)
        {
          joint_pos_[i] = msg->motor_state[i].q;
          joint_vel_[i] = msg->motor_state[i].dq;
          joint_acc_[i] = msg->motor_state[i].ddq;
          joint_tau_[i] = msg->motor_state[i].tau;
        }
      });

  executor_.add_node(node_ptr_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ManipulatorDummyEstimator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn ManipulatorDummyEstimator::on_configure(const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ManipulatorDummyEstimator::update_and_write_commands(const rclcpp::Time& time,
                                                                                    const rclcpp::Duration& period)
{
  executor_.spin_some(std::chrono::milliseconds(1));

  return BaseManipulatorEstimator::update_and_write_commands(time, period);
}

controller_interface::return_type ManipulatorDummyEstimator::update_reference_from_subscribers(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

};  // namespace manipulator_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(manipulator_estimators::ManipulatorDummyEstimator, controller_interface::ChainableControllerInterface);
