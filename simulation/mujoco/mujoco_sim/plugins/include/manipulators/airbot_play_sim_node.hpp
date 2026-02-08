#pragma once
#ifndef AIRBOT_PLAY_SIM_NODE_HPP
#define AIRBOT_PLAY_SIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include "manipulator_msgs/msg/low_state.hpp"
#include "manipulator_msgs/msg/low_cmd.hpp"
#include "manipulator_msgs/msg/manipulator_est.hpp"
#include "end_effector_msgs/msg/gripper_state.hpp"
#include "end_effector_msgs/msg/gripper_cmd.hpp"

#include "array_safety.h"
#include "simulate.h"
#include "mujoco_sim_node_base.hpp"

using namespace std::chrono_literals;

namespace mujoco_sim{

namespace mj = mujoco;

class AirbotPlaySimNode : public MujocoSimNodeBase
{
public:
    // constructor of the node
    AirbotPlaySimNode();

    void reset_params();

protected:
    void callback_low_state();
    void callback_low_cmd(const manipulator_msgs::msg::LowCmd::SharedPtr msg);

    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    rclcpp::Subscription<manipulator_msgs::msg::LowCmd>::SharedPtr cmd_sub_ptr_;
    rclcpp::Publisher<manipulator_msgs::msg::LowState>::SharedPtr low_state_pub_ptr_;

    // sensor readings
    std::vector<float> joint_pos_{};  // 6 or 7
    std::vector<float> joint_vel_{};
    std::vector<float> joint_torque_{};

    // motor commands
    std::vector<float> mode_{};
    std::vector<float> cmd_torque_{};
    std::vector<float> cmd_pos_{};
    std::vector<float> cmd_vel_{};
    std::vector<float> cmd_kp_{};
    std::vector<float> cmd_kd_{};
};

class AirbotPlayWithGripperSimNode : public AirbotPlaySimNode
{
public:
    AirbotPlayWithGripperSimNode();

    void reset_params();

protected:
    void callback_gripper_low_state();
    void callback_gripper_low_cmd(const end_effector_msgs::msg::GripperCmd::SharedPtr msg);

    rclcpp::Subscription<end_effector_msgs::msg::GripperCmd>::SharedPtr gripper_cmd_sub_ptr_;
    rclcpp::Publisher<end_effector_msgs::msg::GripperState>::SharedPtr gripper_state_pub_ptr_;

    std::array<float, 1> gripper_pos_{0.0};
    std::array<float, 1> gripper_vel_{0.0};
    std::array<float, 1> gripper_torque_{0.0};

    std::array<float, 1> gripper_mode_{0.0};
    std::array<float, 1> gripper_cmd_pos_{0.0};
    std::array<float, 1> gripper_cmd_vel_{0.0};
    std::array<float, 1> gripper_cmd_torque_{0.0};
};

class AirbotPlaySimGroundTruth : public AirbotPlaySimNode
{
public:
    AirbotPlaySimGroundTruth();

    void reset_params();

protected:
    void ground_truth_callback();
    rclcpp::Publisher<manipulator_msgs::msg::ManipulatorEst>::SharedPtr manipulator_est_pub_ptr_;

    // ground truth states
    std::array<float, 6> ee_wrench_{0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0}; // end effector wrench
};

} // namespace mujoco_sim

#endif // AIRBOT_PLAY_SIM_NODE_HPP
