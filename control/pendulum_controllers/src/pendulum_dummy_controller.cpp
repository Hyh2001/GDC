#include "pendulum_controllers/pendulum_dummy_controller.hpp"

namespace pendulum_controllers
{

    controller_interface::CallbackReturn PendulumDummyController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PendulumDummyController::command_interface_configuration() const
    {
        // export command interfaces for state estimation
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interface_config.names.push_back(
            ref_controller_name_+"/joint_position");
        command_interface_config.names.push_back(
            ref_controller_name_+"/joint_velocity");
        command_interface_config.names.push_back(
            ref_controller_name_+"/joint_effort");
        command_interface_config.names.push_back(
            ref_controller_name_+"/tip_y");
        command_interface_config.names.push_back(
            ref_controller_name_+"/tip_z");
        command_interface_config.names.push_back(
            ref_controller_name_+"/tip_vy");
        command_interface_config.names.push_back(
            ref_controller_name_+"/tip_vz");
        command_interface_config.names.push_back(
            ref_controller_name_+"/joint_position_ref");
        command_interface_config.names.push_back(
            ref_controller_name_+"/joint_velocity_ref");
        return command_interface_config; // command_interfaces_    
    }

    controller_interface::InterfaceConfiguration PendulumDummyController::state_interface_configuration() const
    {
        // use no state interfaces
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interface_config; // state_interfaces_
    }

    controller_interface::CallbackReturn PendulumDummyController::on_configure(const rclcpp_lifecycle::State &)
    {
        node_name_ = auto_declare<std::string>("node_name", "");
        subscribe_topic_name_ = auto_declare<std::string>("subscribe_topic_name", "");
        publish_topic_name_ = auto_declare<std::string>("publish_topic_name", "");
        ref_controller_name_ = auto_declare<std::string>("ref_controller_name", "");
        if(node_name_.empty() || subscribe_topic_name_.empty())
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(),
                "PendulumDummyController: 'node_name', 'publish_topic_name' or 'subscribe_topic_name' parameters are empty.");
            return controller_interface::CallbackReturn::FAILURE;
        }
        node_ptr_ = rclcpp::Node::make_shared(node_name_);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
        control_subscriber_ = node_ptr_->create_subscription<std_msgs::msg::Float64MultiArray>(
            subscribe_topic_name_, qos,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            {
                joint_new_vel_des_ = msg->data[0];
            });
        state_publisher_ = node_ptr_->create_publisher<std_msgs::msg::Float64MultiArray>(
            publish_topic_name_, qos);
        realtime_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(state_publisher_);
        executor_.add_node(node_ptr_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumDummyController::on_cleanup(const rclcpp_lifecycle::State &)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumDummyController::on_activate(const rclcpp_lifecycle::State &)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumDummyController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> PendulumDummyController::on_export_reference_interfaces()
    {
        if (reference_interfaces_.empty()) {
            // We need to create storage for all references: joints (3) + tip (4) + ref(2)
            // Total: 3 + 4 + 2= 9 values
            reference_interfaces_.resize(9, 0.0);
        }
        std::string controller_name = this->get_node()->get_name();
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        // state estimation
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_position", &reference_interfaces_[0]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_velocity", &reference_interfaces_[1]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_effort", &reference_interfaces_[2]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_y", &reference_interfaces_[3]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_z", &reference_interfaces_[4]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_vy", &reference_interfaces_[5]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "tip_vz", &reference_interfaces_[6]));
        // reference
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_position_ref", &reference_interfaces_[7]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(
            controller_name, "joint_velocity_ref", &reference_interfaces_[8])); 
        return reference_interfaces;
    }

    bool PendulumDummyController::on_set_chained_mode(bool chained_mode)
    {
        return true; // enable chaining since this is a controller and leverage command interface
    }

    controller_interface::return_type PendulumDummyController::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        executor_.spin_some(std::chrono::milliseconds(1));
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(3);
        msg.data[0] = reference_interfaces_[0]; // joint_pos
        msg.data[1] = reference_interfaces_[1]; // joint_vel
        msg.data[2] = reference_interfaces_[8]; // joint_vel_des
        realtime_state_publisher_->lock();
        realtime_state_publisher_->msg_ = msg;
        realtime_state_publisher_->unlockAndPublish();
        command_interfaces_[0].set_value(joint_new_vel_des_);
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type PendulumDummyController::update_reference_from_subscribers(
        const rclcpp::Time &time, const rclcpp::Duration &period
    )
    {
        return controller_interface::return_type::OK;
    }

    
}; // namespace pendulum_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pendulum_controllers::PendulumDummyController, controller_interface::ChainableControllerInterface);