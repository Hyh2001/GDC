#pragma once

#include <vector>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "humanoid_msgs/msg/low_state.hpp"
#include "humanoid_msgs/msg/low_cmd.hpp"

namespace base_hardware_interfaces
{

    class BaseHumanoidHardwareInterfaces : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        BaseHumanoidHardwareInterfaces(const std::string& node_name)
            : rclcpp_lifecycle::LifecycleNode(node_name) {}

        virtual ~BaseHumanoidHardwareInterfaces() = default;

        virtual void read() = 0;
        virtual void write() = 0;
        virtual void reset() = 0;

    protected:
        // low state
        std::vector<double> joint_positions_{};
        std::vector<double> joint_velocities_{};
        std::vector<double> joint_efforts_{};
        std::array<double, 3> gyro_{0.0, 0.0, 0.0};
        std::array<double, 3> accelerometer_{0.0, 0.0, 0.0};
        std::array<double, 4> quat_{1.0, 0.0, 0.0, 0.0}; // wxyz
        std::array<bool, 2> contacts_{false, false};

        // low cmd
        std::vector<double> joint_position_commands_{};
        std::vector<double> joint_velocity_commands_{};
        std::vector<double> joint_effort_commands_{};
        std::vector<double> joint_kp_gains_{};
        std::vector<double> joint_kd_gains_{};

        // subscriber and publisher (lifecycle versions)
        std::vector<rclcpp::TimerBase::SharedPtr> timers_{};
        rclcpp::Subscription<humanoid_msgs::msg::LowCmd>::SharedPtr cmd_sub_ptr_ = nullptr;
        rclcpp_lifecycle::LifecyclePublisher<humanoid_msgs::msg::LowState>::SharedPtr low_state_pub_ptr_ = nullptr;

    };

} // namespace base_hardware_interfaces
