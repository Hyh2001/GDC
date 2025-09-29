#include "include/quadruped_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hardware_interfaces
{
    QuadrupedHardwareInterface::QuadrupedHardwareInterface()
    : hardware_interface::SystemInterface()
    {
    }

    hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
    {   
        // create the node
        node_ptr_ = rclcpp::Node::make_shared("quadruped_hardware_interface");
        LowState_subscriber_ = node_ptr_->create_subscription<quadruped_msgs::msg::LowState>(
            "/quadruped/low_state", 10,
            [this](const quadruped_msgs::msg::LowState::SharedPtr msg) {
                    for(int i = 0; i < 12; i++){
                        joint_pos_[i] = msg->motor_state[i].q;
                        joint_vel_[i] = msg->motor_state[i].dq;
                        joint_acc_[i] = msg->motor_state[i].ddq;
                        joint_tau_[i] = msg->motor_state[i].tau;
                    }
                    for(int i = 0; i < 4; i++){
                        contact_states_[i] = msg->contact_state[i].contact;
                    } 
                    orientation_[0] = msg->imu.orientation.w;
                    orientation_[1] = msg->imu.orientation.x;
                    orientation_[2] = msg->imu.orientation.y;
                    orientation_[3] = msg->imu.orientation.z;
                    gyro_[0] = msg->imu.angular_velocity.x;
                    gyro_[1] = msg->imu.angular_velocity.y;
                    gyro_[2] = msg->imu.angular_velocity.z;
                    accel_[0] = msg->imu.linear_acceleration.x;
                    accel_[1] = msg->imu.linear_acceleration.y;
                    accel_[2] = msg->imu.linear_acceleration.z;
            });
        LowCmd_publisher_ = node_ptr_->create_publisher<quadruped_msgs::msg::LowCmd>("/quadruped/low_cmd", 10);
        realtime_LowCmd_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<quadruped_msgs::msg::LowCmd>>(LowCmd_publisher_);
        executor_.add_node(node_ptr_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }        

    hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn QuadrupedHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        return hardware_interface::SystemInterface::on_init(info);
    }

    std::vector<hardware_interface::StateInterface> QuadrupedHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            for (size_t j = 0; j < joint_state_interface_types_.size(); ++j) {
                double* data_ptr = nullptr;
                if (joint_state_interface_types_[j] == "position") data_ptr = &joint_pos_[i];
                else if (joint_state_interface_types_[j] == "velocity") data_ptr = &joint_vel_[i];
                else if (joint_state_interface_types_[j] == "effort") data_ptr = &joint_tau_[i];
                if (data_ptr) {
                    state_interfaces.emplace_back(joint_names_[i], joint_state_interface_types_[j], data_ptr);
                }
            }
        }

        // IMU state interfaces
        for (size_t i = 0; i < imu_interface_types_.size(); ++i) {
            double* data_ptr = nullptr;
            if (imu_interface_types_[i] == "orientation.w") data_ptr = &orientation_[0];
            else if (imu_interface_types_[i] == "orientation.x") data_ptr = &orientation_[1];
            else if (imu_interface_types_[i] == "orientation.y") data_ptr = &orientation_[2];
            else if (imu_interface_types_[i] == "orientation.z") data_ptr = &orientation_[3];
            else if (imu_interface_types_[i] == "angular_velocity.x") data_ptr = &gyro_[0];
            else if (imu_interface_types_[i] == "angular_velocity.y") data_ptr = &gyro_[1];
            else if (imu_interface_types_[i] == "angular_velocity.z") data_ptr = &gyro_[2];
            else if (imu_interface_types_[i] == "linear_acceleration.x") data_ptr = &accel_[0];
            else if (imu_interface_types_[i] == "linear_acceleration.y") data_ptr = &accel_[1];
            else if (imu_interface_types_[i] == "linear_acceleration.z") data_ptr = &accel_[2];
            if (data_ptr) {
                state_interfaces.emplace_back(imu_name_, imu_interface_types_[i], data_ptr);
            }
        }

        // Contact sensor state interfaces
        for (size_t i = 0; i < contact_sensor_interface_types_.size(); ++i) {
            state_interfaces.emplace_back(contact_sensor_name_, contact_sensor_interface_types_[i], &contact_states_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> QuadrupedHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        // FR
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            for (size_t j = 0; j < joint_command_interface_types_.size(); ++j) {
                double* data_ptr = nullptr;
                if (joint_command_interface_types_[j] == "position") data_ptr = &joint_pos_command_[i];
                else if (joint_command_interface_types_[j] == "velocity") data_ptr = &joint_vel_command_[i];
                else if (joint_command_interface_types_[j] == "effort") data_ptr = &joint_tau_command_[i];
                else if (joint_command_interface_types_[j] == "kp") data_ptr = &joint_kp_[i];
                else if (joint_command_interface_types_[j] == "kd") data_ptr = &joint_kd_[i];
                if (data_ptr) {
                    command_interfaces.emplace_back(joint_names_[i], joint_command_interface_types_[j], data_ptr);
                }
            }
        }
        return command_interfaces;
    }

    hardware_interface::return_type QuadrupedHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        executor_.spin_some(std::chrono::milliseconds(1));
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type QuadrupedHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        auto msg = quadruped_msgs::msg::LowCmd();
        msg.header.stamp = node_ptr_->now();
        for(int i = 0; i < 12; i++){
            msg.motor_cmd[i].q = joint_pos_command_[i];
            msg.motor_cmd[i].dq = joint_vel_command_[i];
            msg.motor_cmd[i].tau = joint_tau_command_[i];
            msg.motor_cmd[i].kp = joint_kp_[i];
            msg.motor_cmd[i].kd = joint_kd_[i];
        }
        // publish the command
        realtime_LowCmd_publisher_->lock();
        realtime_LowCmd_publisher_->msg_ = msg;
        realtime_LowCmd_publisher_->unlockAndPublish();
        // LowCmd_publisher_->publish(msg);
        return hardware_interface::return_type::OK;
    }

};

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(hardware_interfaces::QuadrupedHardwareInterface, 
//             hardware_interface::SystemInterface)