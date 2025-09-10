#include "include/go2_sim_node.hpp"

namespace mujoco_sim{ 

Go2SimNode::Go2SimNode() : MujocoSimNodeBase("go2_sim"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    cmd_sub_ptr_ = this->create_subscription<quadruped_msgs::msg::LowCmd>(
        "/go2/low_cmd", qos, std::bind(&Go2SimNode::callback_low_cmd, this, std::placeholders::_1));

    low_state_pub_ptr_ = this->create_publisher<quadruped_msgs::msg::LowState>("/go2/low_state", qos);

    timers_.emplace_back(this->create_wall_timer(
      2ms, std::bind(&Go2SimNode::callback_low_state, this)));


    reset_params(); 


}

void Go2SimNode::reset_params() {
    // reset all the params
    // sensor readings
    std::fill(std::begin(joint_pos_), std::end(joint_pos_), 0.0);
    std::fill(std::begin(joint_vel_), std::end(joint_vel_), 0.0);
    std::fill(std::begin(gyro_), std::end(gyro_), 0.0);
    std::fill(std::begin(accelerom_), std::end(accelerom_), 0.0);
    std::fill(std::begin(contact_), std::end(contact_), false);

    // motor commands
    std::fill(std::begin(cmd_torque_), std::end(cmd_torque_), 0.0);
    std::fill(std::begin(cmd_pos_), std::end(cmd_pos_), 0.0);
    std::fill(std::begin(cmd_vel_), std::end(cmd_vel_), 0.0);
    // motor params
    std::fill(std::begin(cmd_kp_), std::end(cmd_kp_), 0.0);
    std::fill(std::begin(cmd_kd_), std::end(cmd_kd_), 0.0);
}

void Go2SimNode::callback_low_state() {
    // get the sensor data
    if (sim_ -> d_){
        quadruped_msgs::msg::LowState low_state_msg = quadruped_msgs::msg::LowState();

        // lock the thread
        const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

        // get the data
        const int idx_imu_gyro = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "imu_gyro")];
        const int idx_imu_accele = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "imu_acc")];
        const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_pos")];
        const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_vel")];
        const int idx_joint_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_torque")];
        const int idx_contact = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_contact_sensor")];
        
        low_state_msg.imu.linear_acceleration.x = sim_->d_->sensordata[idx_imu_accele + 0];
        low_state_msg.imu.linear_acceleration.y = sim_->d_->sensordata[idx_imu_accele + 1];
        low_state_msg.imu.linear_acceleration.z = sim_->d_->sensordata[idx_imu_accele + 2];
        low_state_msg.imu.angular_velocity.x = sim_->d_->sensordata[idx_imu_gyro + 0];
        low_state_msg.imu.angular_velocity.y = sim_->d_->sensordata[idx_imu_gyro + 1];
        low_state_msg.imu.angular_velocity.z = sim_->d_->sensordata[idx_imu_gyro + 2];

        for (int i = 0; i < 12; i++){
            low_state_msg.motor_state[i].q = sim_->d_->sensordata[idx_joint_pos + i];
            low_state_msg.motor_state[i].dq = sim_->d_->sensordata[idx_joint_vel + i];
            low_state_msg.motor_state[i].tau = sim_->d_->sensordata[idx_joint_torque + i];
        }
        bool contact; 
        for (int i = 0; i < 4; i++){
            if(sim_->d_->sensordata[idx_contact + i] > 0.0) {
                contact = true; 
            }
            else {
                contact = false;     
            }
            low_state_msg.contact_state[i].contact = contact;
        }

        // publish
        double sim_time = sim_->d_->time;
        low_state_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
        low_state_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - low_state_msg.header.stamp.sec) * 1e9);
        low_state_msg.header.frame_id = "sim_time"; 

        low_state_pub_ptr_->publish(low_state_msg);
    }

}

void Go2SimNode::callback_low_cmd(const quadruped_msgs::msg::LowCmd::SharedPtr msg) {
    if(sim_ ->  d_){
        // const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
        // apply the motor commands
        const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_pos")];
        const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_vel")];
        for (int i = 0; i < 12; i++){
            sim_->d_->ctrl[i] = msg->motor_cmd[i].tau 
                + msg->motor_cmd[i].kp * (msg->motor_cmd[i].q - sim_->d_->sensordata[idx_joint_pos + i])
                + msg->motor_cmd[i].kd * (msg->motor_cmd[i].dq - sim_->d_->sensordata[idx_joint_vel + i]);
        }
    }
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_sim::Go2SimNode, mujoco_sim::MujocoSimNodeBase)
// PLUGINLIB_EXPORT_CLASS(mujoco_sim::Go2SimGroundTruth, mujoco_sim::MujocoSimNodeBase)
