#ifndef PENDULUM_SIM_NODE_HPP
#define PENDULUM_SIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include "pendulum_msgs/msg/low_state.hpp"
#include "pendulum_msgs/msg/low_cmd.hpp"
#include "pendulum_msgs/msg/pendulum_est.hpp"

#include "array_safety.h"
#include "simulate.h"
#include "mujoco_sim_node_base.hpp"

using namespace std::chrono_literals; 

namespace mujoco_sim{

namespace mj = mujoco;

class PendulumSimNode : public MujocoSimNodeBase{
public:
    // constructor of the node
    PendulumSimNode();

    void reset_params();

protected: 
    void callback_low_state();
    void callback_low_cmd(const pendulum_msgs::msg::LowCmd::SharedPtr msg);

    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    rclcpp::Subscription<pendulum_msgs::msg::LowCmd>::SharedPtr cmd_sub_ptr_;
    rclcpp::Publisher<pendulum_msgs::msg::LowState>::SharedPtr low_state_pub_ptr_;
    
    // sensor readings
    float joint_pos_;
    float joint_vel_;
    float joint_accel_;
    float joint_torque_;

    // motor commands
    float cmd_torque_;
    float cmd_pos_; 
    float cmd_vel_;
    // motor params
    float cmd_kp_;
    float cmd_kd_;
};

class PendulumSimGroundTruth : public PendulumSimNode{
public:

    PendulumSimGroundTruth();

    void reset_params();

protected: 
    void ground_truth_callback();
    rclcpp::Publisher<pendulum_msgs::msg::PendulumEst>::SharedPtr pendulum_est_pub_ptr_;

    // ground truth
    std::array<float, 4> tip_state_{0.0, 0.0, 0.0, 0.0}; // y, vy, z, vz  
};


} // namespace mujoco_sim

#endif 