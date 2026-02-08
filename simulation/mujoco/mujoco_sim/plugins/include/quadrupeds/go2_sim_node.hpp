#ifndef GO2_SIM_NODE_HPP
#define GO2_SIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include "quadruped_msgs/msg/low_state.hpp"
#include "quadruped_msgs/msg/low_cmd.hpp"
#include "quadruped_msgs/msg/quad_est.hpp"

#include "array_safety.h"
#include "simulate.h"
#include "mujoco_sim_node_base.hpp"

using namespace std::chrono_literals;

namespace mujoco_sim
{

namespace mj = mujoco;

class Go2SimNode : public MujocoSimNodeBase
{
public:
  // constructor of the node
  Go2SimNode();

  void reset_params();

protected:
  void callback_low_state();
  void callback_low_cmd(const quadruped_msgs::msg::LowCmd::SharedPtr msg);

  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Subscription<quadruped_msgs::msg::LowCmd>::SharedPtr cmd_sub_ptr_;
  rclcpp::Publisher<quadruped_msgs::msg::LowState>::SharedPtr low_state_pub_ptr_;

  // sensor readings
  // FR -> FL -> RR -> RL
  std::array<float, 12> joint_pos_;
  std::array<float, 12> joint_vel_;
  std::array<float, 12> joint_torque_;
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

class Go2SimGroundTruth : public Go2SimNode
{
public:
  Go2SimGroundTruth();

  void reset_params();

protected:
  void ground_truth_callback();
  rclcpp::Publisher<quadruped_msgs::msg::QuadEst>::SharedPtr ground_truth_pub_ptr_;

  // ground truth
  std::array<float, 3> pos_truth_{ 0.0, 0.0, 0.0 };        // x, y, z in the world frame
  std::array<float, 4> ori_truth_{ 1.0, 0.0, 0.0, 0.0 };   // w, x, y, z
  std::array<float, 3> lin_vel_truth_{ 0.0, 0.0, 0.0 };    // vx, vy, vz in the world frame
  std::array<float, 3> lin_accel_truth_{ 0.0, 0.0, 0.0 };  // ax, ay, az  in the world frame
  std::array<float, 3> ang_vel_truth_{ 0.0, 0.0, 0.0 };    // wx, wy, wz in the world frame
  std::array<float, 3> ang_acc_truth_{ 0.0, 0.0, 0.0 };    // alphax, alphay, alphaz in the world frame
  std::array<std::array<float, 3>, 4> contact_forces_{
    { { { 0.0f, 0.0f, 0.0f } }, { { 0.0f, 0.0f, 0.0f } }, { { 0.0f, 0.0f, 0.0f } }, { { 0.0f, 0.0f, 0.0f } } }
  };  // fx, fy, fz in the world frame
};

}  // namespace mujoco_sim

#endif
