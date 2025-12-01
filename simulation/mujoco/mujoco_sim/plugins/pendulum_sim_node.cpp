#include "include/pendulunm_sim_node.hpp"

namespace mujoco_sim{ 

PendulumSimNode::PendulumSimNode() : MujocoSimNodeBase("pendulum_sim"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    cmd_sub_ptr_ = this->create_subscription<pend_msgs::msg::LowCmd>(
        "/pendulum/low_cmd", qos, std::bind(&PendulumSimNode::callback_low_cmd, this, std::placeholders::_1));

    low_state_pub_ptr_ = this->create_publisher<pend_msgs::msg::LowState>("/pendulum/low_state", qos);

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
        pend_msgs::msg::LowState low_state_msg = pend_msgs::msg::LowState();

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
        low_state_msg.motor_state.q = sim_->d_->sensordata[idx_joint_pos];
        low_state_msg.motor_state.dq = sim_->d_->sensordata[idx_joint_vel];
        low_state_msg.motor_state.ddq = sim_->d_->sensordata[idx_joint_accel];
        low_state_msg.motor_state.tau = sim_->d_->sensordata[idx_joint_torque];

        low_state_pub_ptr_->publish(low_state_msg);
    }

}

void PendulumSimNode::callback_low_cmd(const pend_msgs::msg::LowCmd::SharedPtr msg) {
    if(sim_ ->  d_){
        // const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
        // apply the motor commands
        const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_position")];
        const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_velocity")];
        sim_ ->d_->ctrl[0] = msg->motor_cmd.tau 
                + msg->motor_cmd.kp * (msg->motor_cmd.q - sim_->d_->sensordata[idx_joint_pos])
                + msg->motor_cmd.kd * (msg->motor_cmd.dq - sim_->d_->sensordata[idx_joint_vel]);
    }
}

PendulumSimGroundTruth::PendulumSimGroundTruth() : PendulumSimNode(){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    pendulum_est_pub_ptr_ = this->create_publisher<pend_msgs::msg::PendulumEst>(
        "/pendulum/pendulum_est", qos);

    timers_.emplace_back(this->create_wall_timer(
      2ms, std::bind(&PendulumSimGroundTruth::ground_truth_callback, this)));

    reset_params(); 
}

void PendulumSimGroundTruth::reset_params() {
    PendulumSimNode::reset_params();
    tip_state_ = {0.0, 0.0, 0.0, 0.0};
}

void PendulumSimGroundTruth::ground_truth_callback() {
    pend_msgs::msg::PendulumEst pendulum_est_msg = pend_msgs::msg::PendulumEst();
    
    if (sim_ -> d_){
        // lock the thread
        const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
        // get the sensor ids
        const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_position")];
        const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_velocity")];
        const int idx_joint_accel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_acceleration")];
        const int idx_joint_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "joint_torque")];
        const int idx_tip_position = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "tip_position")];
        const int idx_tip_velocity = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "tip_velocity")];

        // publish
        double sim_time = sim_->d_->time;
        pendulum_est_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
        pendulum_est_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - pendulum_est_msg.header.stamp.sec) * 1e9);
        pendulum_est_msg.header.frame_id = "sim_time"; 
        pendulum_est_msg.motor_state.q = sim_->d_->sensordata[idx_joint_pos];
        pendulum_est_msg.motor_state.dq = sim_->d_->sensordata[idx_joint_vel];
        pendulum_est_msg.motor_state.ddq = sim_->d_->sensordata[idx_joint_accel];
        pendulum_est_msg.motor_state.tau = sim_->d_->sensordata[idx_joint_torque];
        pendulum_est_msg.tip_state[0] = sim_->d_->sensordata[idx_tip_position + 0];
        pendulum_est_msg.tip_state[1] = sim_->d_->sensordata[idx_tip_position + 2] - 1.5; // put the joint center to 0.0m        pendulum_est_msg.tip_state[2] = sim_->d_->sensordata[idx_tip_velocity + 0];
        pendulum_est_msg.tip_state[3] = sim_->d_->sensordata[idx_tip_velocity + 2];

        pendulum_est_pub_ptr_->publish(pendulum_est_msg);
    }
}

}; // namespace mujoco_sim

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_sim::PendulumSimNode, mujoco_sim::MujocoSimNodeBase)
PLUGINLIB_EXPORT_CLASS(mujoco_sim::PendulumSimGroundTruth, mujoco_sim::MujocoSimNodeBase)
