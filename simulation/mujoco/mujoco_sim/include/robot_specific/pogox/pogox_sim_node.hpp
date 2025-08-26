// #ifndef POGOX_SIM_NODE_H
// #define POGOX_SIM_NODE_H

// #include <rclcpp/rclcpp.hpp>
// #include <rmw/types.h>
// #include "sensor_msgs/msg/imu.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "pogox_communication/msg/actuator_commands.hpp"
// #include "pogox_communication/msg/leg_joint_state.hpp"
// #include "pogox_communication/msg/contact_force.hpp"
// #include "pogox_communication/msg/pogox_state.hpp"

// #include "array_safety.h"
// #include "simulate.h"

// using namespace std::chrono_literals; 

// namespace mujoco_sim {

// namespace mj = mujoco;

// class PogoXSimNode : public rclcpp::Node{
// public:
//     // constructor of the node
//     PogoXSimNode(mj::Simulate *sim);

//     void reset_params();

// protected:
//     void motor_cmd_callback(const pogox_communication::msg::ActuatorCommands::SharedPtr msg);
    
//     void imu_callback();
//     void contact_callback();
//     void leg_joint_state_callback();

//     // mujoco sim ptr
//     mj::Simulate *sim_;

//     // timers, sub & pub
//     std::vector<rclcpp::TimerBase::SharedPtr> timers_;
//     // thrust force subscriber
//     rclcpp::Subscription<pogox_communication::msg::ActuatorCommands>::SharedPtr cmd_sub_ptr_;
//     // publishers
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_state_pub_ptr_;
//     // rclcpp::Publisher<pogox_communication::msg::ContactForce>::SharedPtr contact_force_pub_ptr_;
//     rclcpp::Publisher<pogox_communication::msg::LegJointState>::SharedPtr leg_joint_state_pub_ptr_;
//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_ptr_;

//     // sensor reading
//     float leg_joint_pos_, leg_joint_vel_;
//     std::array<float, 3> gyro_;  
//     std::array<float, 3> accelerom_;
//     std::array<float, 4> orien_; // wxyz
//     bool contact;
//     std::array<float, 3> grf_;

//     // motor commands
//     std::array<float, 4> cmd_force_;
//     // motor params
//     std::array<float, 4> cmd_kp_;
//     std::array<float, 4> cmd_kd_; 
// };

// class PogoXSimGroundTruth: public PogoXSimNode{
// public:
//     PogoXSimGroundTruth(mj::Simulate *sim);

//     void reset_params();

// protected: 
//     void ground_truth_callback();

//     rclcpp::Publisher<pogox_communication::msg::PogoxState>::SharedPtr ground_truth_pub_ptr_;

//     // ground truth data for debug and control test
//     std::array<float, 3> pos_truth_;
//     std::array<float, 3> vel_truth_;
//     std::array<float, 3> acc_truth_;
//     std::array<float, 4> orien_truth_; // wxyz
//     std::array<float, 3> angular_vel_truth_;
//     std::array<float, 3> angular_acc_truth_;
//     std::array<float, 2> leg_joint_state_truth_; 
//     mjtNum contact_frame_truth_[4]; 
//     mjtNum grf_truth_local_[3];
//     mjtNum grf_truth_world_[3];
//     bool contact_truth_;
// };



// } // namespace mujoco_sim

// #endif 