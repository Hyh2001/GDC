#include "pendulum_controllers/pendulum_velocity_policy.hpp"

namespace pendulum_controllers
{

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PendulumVelocityPolicy::command_interface_configuration() const
    {
        // export command interfaces for state estimation
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interface_config.names.push_back(
            std::string("pivot/")+"position");
        command_interface_config.names.push_back(
            std::string("pivot/")+"kp");
        command_interface_config.names.push_back(
            std::string("pivot/")+"kd");
        return command_interface_config; // command_interfaces_    
    }

    controller_interface::InterfaceConfiguration PendulumVelocityPolicy::state_interface_configuration() const
    {
        // use no state interfaces
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

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_configure(const rclcpp_lifecycle::State &)
    {
        estimator_name_ = auto_declare<std::string>("estimator_name", "estimator");
        planner_name_ = auto_declare<std::string>("planner_name", "planner");
        std::string policy_path = auto_declare<std::string>("policy_path", "");
        int input_size = auto_declare<int>("input_size", 3);
        int output_size = auto_declare<int>("output_size", 1);
        kp_ = auto_declare<double>("kp", 0.0);
        kd_ = auto_declare<double>("kd", 0.0);
        velocity_policy_ptr_ = std::make_shared<base_controllers::OnnxPolicy>(policy_path, input_size, output_size);
        input_vector_.resize(input_size, 0.0);
        output_vector_.resize(output_size, 0.0);
        
        // debug related
        debug_ = auto_declare<bool>("debug", false);
        if (debug_){
            logger_ptr_ = std::make_unique<loggers::Logger>(
                "pendulum_velocity_policy_logger", 5
            ); 
            loggers::ValueConfig config;
            config.source_name = "pendulum_velocity_policy"; 
            config.labels.resize(2); 
            config.labels[0] = "velocity_ref";
            config.labels[1] = "velocity_real";
            config.value_ptrs.resize(2);
            config.value_ptrs[0] = &input_vector_[2]; // velocity_ref
            config.value_ptrs[1] = &input_vector_[1]; // velocity_real
            logger_ptr_->register_values(config);    
        };
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if(debug_ && logger_ptr_){
            logger_ptr_->stop();
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_activate(const rclcpp_lifecycle::State &)
    {
        if(debug_ && logger_ptr_){
            logger_ptr_->start();
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PendulumVelocityPolicy::on_deactivate(const rclcpp_lifecycle::State &)
    {
        if(debug_ && logger_ptr_){
            logger_ptr_->pause();
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PendulumVelocityPolicy::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // if ((time - last_infer_time_).nanoseconds() * 1e-9 < infer_period_) {
        //         return controller_interface::return_type::OK;
        // }
        // last_infer_time_ = time;
        // get the current state
        double position = state_interfaces_[0].get_value(); // joint_position
        double velocity = state_interfaces_[1].get_value(); // joint_velocity
        double position_ref = state_interfaces_[7].get_value(); // joint_position_ref
        double velocity_ref = state_interfaces_[8].get_value(); // joint_velocity_ref
        // std::cout << "PendulumPID: pos_ref=" << position_ref << ", pos=" << position << std::endl;
        input_vector_[0] = position;
        input_vector_[1] = velocity; 
        input_vector_[2] = velocity_ref;
        output_vector_ = velocity_policy_ptr_->infer(input_vector_);
        if (output_vector_.size() != 1){
            RCLCPP_ERROR(this->get_node()->get_logger(), "PendulumVelocityPolicy: output size mismatch, expect only one position!");

            return controller_interface::return_type::ERROR;
        }
        // write to command interface
        command_interfaces_[0].set_value(output_vector_[0]); // effort
        command_interfaces_[1].set_value(kp_); // kp
        command_interfaces_[2].set_value(kd_); // kd
        
        return controller_interface::return_type::OK;
    }
    
}; // namespace pendulum_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pendulum_controllers::PendulumVelocityPolicy, controller_interface::ControllerInterface);