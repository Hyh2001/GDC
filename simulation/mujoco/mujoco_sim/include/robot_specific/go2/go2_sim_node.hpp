#ifndef GO2_SIM_NODE_HPP
#define GO2_SIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include "quadruped_msgs/msg/low_state.hpp"
#include "quadruped_msgs/msg/low_cmd.hpp"
#include "quadruped_msgs/msg/quad_est.hpp"

#include "../../array_safety.h"
#include "../../simulate.h"

using namespace std::chrono_literals; 

namespace mujoco_sim{

namespace mj = mujoco;

class Go2SimNode : public rclcpp::Node{
public:
    // constructor of the node
    Go2SimNode(mj::Simulate *sim);

    void reset_params();

protected: 
    void callback_low_state();
    void callback_low_cmd(const quadruped_msgs::msg::LowCmd::SharedPtr msg);

    mj::Simulate *sim_;

    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    rclcpp::Subscription<quadruped_msgs::msg::LowCmd>::SharedPtr cmd_sub_ptr_;
    rclcpp::Publisher<quadruped_msgs::msg::LowState>::SharedPtr low_state_pub_ptr_;
    
    // sensor readings
    std::array<float, 12> joint_pos_;
    std::array<float, 12> joint_vel_;
    std::array<float, 3> gyro_;
    std::array<float, 3> accelerom_;
    std::array<bool, 4> contact_; 

    // motor commands
    std::array<float, 12> cmd_torque_;
    std::array<float, 12> cmd_pos_; 
    std::array<float, 12> cmd_vel_;
    // motor params
    std::array<float, 12> cmd_kp_;
    std::array<float, 12> cmd_kd_;
};

class Go2SimGroundTruth : public Go2SimNode{
public:
    Go2SimGroundTruth(mj::Simulate *sim);

    void reset_params();

protected: 
    void ground_truth_callback();
    rclcpp::Publisher<quadruped_msgs::msg::QuadEst>::SharedPtr ground_truth_pub_ptr_;
};


} // namespace mujoco_sim
#endif 