#include "include/pendulunm_sim_node.hpp"

namespace mujoco_sim{ 

PendulumSimNode::PendulumSimNode() : MujocoSimNodeBase("pendulum_sim"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    cmd_sub_ptr_ = this->create_subscription<pendulum_msgs::msg::LowCmd>(
        "/Pendulum/low_cmd", qos, std::bind(&PendulumSimNode::callback_low_cmd, this, std::placeholders::_1));

    low_state_pub_ptr_ = this->create_publisher<pendulum_msgs::msg::LowState>("/Pendulum/low_state", qos);

    timers_.emplace_back(this->create_wall_timer(
      2ms, std::bind(&PendulumSimNode::callback_low_state, this)));


    reset_params(); 


}

void PendulumSimNode::reset_params() {
    // reset all the params
    // sensor readings
    joint_pos_ = 0.0;
    joint_vel_ = 0.0;
    joint_accel_ = 0.0;
    joint_torque_ = 0.0;

    // motor commands
    cmd_torque_ = 0.0;
    cmd_pos_ = 0.0; 
    cmd_vel_ = 0.0;
    // motor params
    cmd_kp_ = 0.0;
    cmd_kd_ = 0.0;
}

void PendulumSimNode::callback_low_state() {
    // get the sensor data
    if (sim_ -> d_){
        pendulum_msgs::msg::LowState low_state_msg = pendulum_msgs::msg::LowState();

        // lock the thread
        const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

        // get the data
        const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_position")];
        const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_velocity")];
        const int idx_joint_accel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_acceleration")];
        const int idx_joint_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_torque")];

        // publish
        double sim_time = sim_->d_->time;
        low_state_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
        low_state_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg.header.stamp.sec) * 1e9);
        low_state_msg.header.frame_id = "sim_time"; 

        low_state_pub_ptr_->publish(low_state_msg);
    }

}

void PendulumSimNode::callback_low_cmd(const pendulum_msgs::msg::LowCmd::SharedPtr msg) {
    if(sim_ ->  d_){
        // const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
        // apply the motor commands
        const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_position")];
        const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_velocity")];
        sim_ ->d_->ctrl[0] = msg->motor_cmd.tau 
                + msg->motor_cmd.kp * (msg->motor_cmd.q - sim_->d_->sensordata[idx_joint_pos])
                + msg->motor_cmd.kd * (msg->motor_cmd.dq - sim_->d_->sensordata[idx_joint_vel]);
        // for (int i = 0; i < 12; i++){
        //     sim_->d_->ctrl[i] = msg->motor_cmd[i].tau 
        //         + msg->motor_cmd[i].kp * (msg->motor_cmd[i].q - sim_->d_->sensordata[idx_joint_pos + i])
        //         + msg->motor_cmd[i].kd * (msg->motor_cmd[i].dq - sim_->d_->sensordata[idx_joint_vel + i]);
        // }
    }
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_sim::PendulumSimNode, mujoco_sim::MujocoSimNodeBase)
// PLUGINLIB_EXPORT_CLASS(mujoco_sim::PendulumSimGroundTruth, mujoco_sim::MujocoSimNodeBase)
