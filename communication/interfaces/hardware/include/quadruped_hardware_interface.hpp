#ifndef QUADRUPED_HARDWARE_INTERFACE_HPP
#define QUADRUPED_HARDWARE_INTERFACE_HPP

#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "quadruped_msgs/msg/low_state.hpp"
#include "quadruped_msgs/msg/low_cmd.hpp"
#include "quadruped_msgs/msg/quad_est.hpp"

namespace hardware_interfaces
{
    class QuadrupedHardwareInterface : public hardware_interface::SystemInterface
    {
    public: 
        QuadrupedHardwareInterface();

        virtual hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        virtual hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        virtual hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    protected:
        std::string node_name_ = "quadruped_hardware_interface";
        std::string publish_topic_name_ = "/quadruped/low_cmd";
        std::string subscribe_topic_name_ = "/quadruped/low_state";
        // FR FL RR RL
        std::array<std::string, 12> joint_names_ = {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        };
        std::array<std::string, 3> joint_state_interface_types_ = {"position", "velocity", "effort"};
        std::array<std::string, 5> joint_command_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
        std::string imu_name_ = "imu";
        std::vector<std::string> imu_interface_types_ = {"orientation.w","orientation.x", "orientation.y", "orientation.z",
                                                         "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",  
                                                         "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
        std::string contact_sensor_name_ = "foot_contact_sensor";
        std::vector<std::string> contact_sensor_interface_types_ = {"FR", "FL", "RR", "RL"};

        // state interfaces
        std::array<double, 12> joint_pos_;
        std::array<double, 12> joint_vel_;
        std::array<double, 12> joint_acc_;
        std::array<double, 12> joint_tau_;
        std::array<double, 4> contact_states_;
        std::array<double, 4> orientation_; // quaternion w,x,y,z
        std::array<double, 3> gyro_; 
        std::array<double, 3> accel_;
        // command interfaces
        std::array<double, 12> joint_pos_command_;
        std::array<double, 12> joint_vel_command_;
        std::array<double, 12> joint_tau_command_;
        std::array<double, 12> joint_kp_;
        std::array<double, 12> joint_kd_;
    
        rclcpp::Node::SharedPtr node_ptr_ = nullptr;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Subscription<quadruped_msgs::msg::LowState>::SharedPtr LowState_subscriber_ = nullptr;
        rclcpp::Publisher<quadruped_msgs::msg::LowCmd>::SharedPtr LowCmd_publisher_ = nullptr;
        realtime_tools::RealtimePublisher<quadruped_msgs::msg::LowCmd>::SharedPtr realtime_LowCmd_publisher_ = nullptr;
    }; 

}; 


#endif // QUADRUPED_HARDWARE_INTERFACE_HPP