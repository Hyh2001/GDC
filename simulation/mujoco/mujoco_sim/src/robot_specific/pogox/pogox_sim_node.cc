#include "pogox_sim_node.h"

namespace mujoco_sim {

// basic sim node

PogoXSimNode::PogoXSimNode(mj::Simulate *sim) : Node("pogox_sim"), sim_(sim) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    cmd_sub_ptr_ = this->create_subscription<pogox_communication::msg::ActuatorCommands>(
        "/pogox/thrust_force_cmd", qos, std::bind(&PogoXSimNode::motor_cmd_callback, this, std::placeholders::_1));

    contact_state_pub_ptr_ = this->create_publisher<std_msgs::msg::Bool>("/pogox/contact", qos);
    // contact_force_pub_ptr_ = this->create_publisher<pogox_communication::msg::ContactForce>("/pogox/contact_force", qos);
    leg_joint_state_pub_ptr_ = this->create_publisher<pogox_communication::msg::LegJointState>("/pogox/leg", qos);
    imu_pub_ptr_ = this->create_publisher<sensor_msgs::msg::Imu>("/pogox/imu", qos);

    timers_.emplace_back(this->create_wall_timer(
      2ms, std::bind(&PogoXSimNode::imu_callback, this)));
    timers_.emplace_back(this->create_wall_timer(
      2ms, std::bind(&PogoXSimNode::contact_callback, this)));
    timers_.emplace_back(this->create_wall_timer(
      2ms, std::bind(&PogoXSimNode::leg_joint_state_callback, this)));  
    reset_params(); 
}

void PogoXSimNode::reset_params() {
    // reset all the params
    leg_joint_pos_ = 0.0;
    leg_joint_vel_ = 0.0; 
    std::fill(std::begin(gyro_), std::end(gyro_), 0.0);
    std::fill(std::begin(accelerom_), std::end(accelerom_), 0.0);
    std::fill(std::begin(orien_), std::end(orien_), 0.0);
    orien_[0] = 1.0; // w of quaternion is 1.0 

    std::fill(std::begin(grf_), std::end(grf_), 0.0);

    std::fill(std::begin(cmd_force_), std::end(cmd_force_), 0.0);
    std::fill(std::begin(cmd_kp_), std::end(cmd_kp_), 0.0);
    std::fill(std::begin(cmd_kd_), std::end(cmd_kd_), 0.0);
}

void PogoXSimNode::imu_callback() {
    if (sim_->d_) {
        // TODO: add time stamp
        
        sensor_msgs::msg::Imu imu_msg_ = sensor_msgs::msg::Imu();
        
        const int idx_imu_quat = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "body_quat")];
        const int idx_imu_gyro = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "body_gyro")];
        const int idx_imu_accele = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "body_acc")];

        // get the data
        orien_[0] = sim_->d_->sensordata[idx_imu_quat];
        orien_[1] = sim_->d_->sensordata[idx_imu_quat+1];
        orien_[2] = sim_->d_->sensordata[idx_imu_quat+2];
        orien_[3] = sim_->d_->sensordata[idx_imu_quat+3];
        imu_msg_.orientation.w = orien_[0];
        imu_msg_.orientation.x = orien_[1];
        imu_msg_.orientation.y = orien_[2];
        imu_msg_.orientation.z = orien_[3];

        gyro_[0] = sim_->d_->sensordata[idx_imu_gyro];
        gyro_[1] = sim_->d_->sensordata[idx_imu_gyro+1];    
        gyro_[2] = sim_->d_->sensordata[idx_imu_gyro+2];
        imu_msg_.angular_velocity.x = gyro_[0];
        imu_msg_.angular_velocity.y = gyro_[1];
        imu_msg_.angular_velocity.z = gyro_[2];
        
        accelerom_[0] = sim_->d_->sensordata[idx_imu_accele];
        accelerom_[1] = sim_->d_->sensordata[idx_imu_accele+1];
        accelerom_[2] = sim_->d_->sensordata[idx_imu_accele+2];
        imu_msg_.linear_acceleration.x = accelerom_[0];
        imu_msg_.linear_acceleration.y = accelerom_[1];
        imu_msg_.linear_acceleration.z = accelerom_[2];

        // publish the data
        imu_pub_ptr_->publish(imu_msg_);  
    }

}

void PogoXSimNode::contact_callback() {
    // TODO: add time stamp
    if (sim_->d_) {

        std_msgs::msg::Bool contact_msg_ = std_msgs::msg::Bool();

        const int idx_contact_bool = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "contact")];

        // get the data
        if(sim_->d_->sensordata[idx_contact_bool] > 0.0) {
            contact = true; 
        }
        else {
            contact = false;     
        }
        contact_msg_.data = contact; 
        
        // publish the data  
        contact_state_pub_ptr_->publish(contact_msg_);
    }

}

void PogoXSimNode::leg_joint_state_callback() {
    // TODO: add time stamp

    if (sim_->d_) {
        //TODO: add time stamp

        pogox_communication::msg::LegJointState leg_joint_state_msg_ = pogox_communication::msg::LegJointState();

        // find leg joint sensor idx
        const int idx_leg_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "leg_joint_pos")];
        const int idx_leg_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "leg_joint_vel")];

        // publish the data
        leg_joint_pos_ = sim_->d_->sensordata[idx_leg_joint_pos];
        leg_joint_vel_ = sim_->d_->sensordata[idx_leg_joint_vel];
        leg_joint_state_msg_.leg_state[0] = leg_joint_pos_;
        leg_joint_state_msg_.leg_state[1] = leg_joint_vel_;
        leg_joint_state_pub_ptr_->publish(leg_joint_state_msg_);
    }

}  

void PogoXSimNode::motor_cmd_callback(const pogox_communication::msg::ActuatorCommands::SharedPtr msg) {
    
    if (sim_->d_ ) {
        // receive data
        for(int i = 0; i < 4; i++) {
            // cmd_msg_->thrust_force[i] = msg->thrust_force[i];
            cmd_force_[i] = msg->thrust_force[i];
        }

        // apply thrust force
        if(sim_->d_ != nullptr) {
            for(int i = 0; i < 4; i++) {
                sim_->d_->ctrl[i] = cmd_force_[i];
            }
        } 
    }
   
}   


// sim node with ground truth data
PogoXSimGroundTruth::PogoXSimGroundTruth(mj::Simulate *sim): PogoXSimNode(sim) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

    ground_truth_pub_ptr_ = this->create_publisher<pogox_communication::msg::PogoxState>("/pogox/estimation", qos);
    
    timers_.emplace_back(this->create_wall_timer(
      2ms, std::bind(&PogoXSimGroundTruth::ground_truth_callback, this)));

    reset_params(); 
}

void PogoXSimGroundTruth::reset_params(){
    PogoXSimNode::reset_params(); 
    //reset ground truth
    std::fill(std::begin(pos_truth_), std::end(pos_truth_), 0.0);
    std::fill(std::begin(vel_truth_), std::end(vel_truth_), 0.0);
    std::fill(std::begin(acc_truth_), std::end(acc_truth_), 0.0);
    std::fill(std::begin(orien_truth_), std::end(orien_truth_), 0.0);
    orien_truth_[0] = 1.0; // w of quaternion is 1.0
    std::fill(std::begin(angular_vel_truth_), std::end(angular_vel_truth_), 0.0);
    std::fill(std::begin(angular_acc_truth_), std::end(angular_acc_truth_), 0.0);
}

void PogoXSimGroundTruth::ground_truth_callback() {
    if (sim_->d_) {
        // update ground truth data

        const int idx_pos_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "pos_ground_truth")];
        const int idx_quat_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "quat_ground_truth")];
        const int idx_linear_vel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "linear_velo_ground_truth")];
        const int idx_angular_vel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "angular_velo_ground_truth")];
        const int idx_linear_accel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "linear_accel_ground_truth")];
        const int idx_angular_accel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "angular_accel_ground_truth")];
        const int idx_leg_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "leg_joint_pos")];
        const int idx_leg_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "leg_joint_vel")];
        const int idx_contact_bool = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "contact")];
        const int idx_contact_force = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "grf")];
        const int idx_contact_frame = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "foot_quat")];
        
        // position
        pos_truth_[0] = sim_->d_->sensordata[idx_pos_truth];
        pos_truth_[1] = sim_->d_->sensordata[idx_pos_truth + 1];
        pos_truth_[2] = sim_->d_->sensordata[idx_pos_truth + 2];
        orien_truth_[0] = sim_->d_->sensordata[idx_quat_truth]; // w
        orien_truth_[1] = sim_->d_->sensordata[idx_quat_truth + 1]; // x
        orien_truth_[2] = sim_->d_->sensordata[idx_quat_truth + 2]; // y
        orien_truth_[3] = sim_->d_->sensordata[idx_quat_truth + 3]; // z
        // velocity 
        vel_truth_[0] = sim_->d_->sensordata[idx_linear_vel_truth];
        vel_truth_[1] = sim_->d_->sensordata[idx_linear_vel_truth + 1];
        vel_truth_[2] = sim_->d_->sensordata[idx_linear_vel_truth + 2];
        angular_vel_truth_[0] = sim_->d_->sensordata[idx_angular_vel_truth];
        angular_vel_truth_[1] = sim_->d_->sensordata[idx_angular_vel_truth + 1];
        angular_vel_truth_[2] = sim_->d_->sensordata[idx_angular_vel_truth + 2];
        // acceleration
        acc_truth_[0] = sim_->d_->sensordata[idx_linear_accel_truth];
        acc_truth_[1] = sim_->d_->sensordata[idx_linear_accel_truth + 1];
        acc_truth_[2] = sim_->d_->sensordata[idx_linear_accel_truth + 2];
        angular_acc_truth_[0] = sim_->d_->sensordata[idx_angular_accel_truth];  
        angular_acc_truth_[1] = sim_->d_->sensordata[idx_angular_accel_truth + 1];
        angular_acc_truth_[2] = sim_->d_->sensordata[idx_angular_accel_truth + 2];
        // joint state
        leg_joint_state_truth_[0] = sim_->d_->sensordata[idx_leg_joint_pos];
        leg_joint_state_truth_[1] = sim_->d_->sensordata[idx_leg_joint_vel];
        // contact state
        if(sim_->d_->sensordata[idx_contact_bool] > 0.0) {
            contact = true; 
        }
        else {
            contact = false;     
        }
        contact_truth_ = contact; 
        // contact force
        contact_frame_truth_[0] = sim_->d_->sensordata[idx_contact_frame];
        contact_frame_truth_[1] = sim_->d_->sensordata[idx_contact_frame + 1];
        contact_frame_truth_[2] = sim_->d_->sensordata[idx_contact_frame + 2];
        contact_frame_truth_[3] = sim_->d_->sensordata[idx_contact_frame + 3];
        // flip the value on dummy object for real contact force
        // TODO: correct the frame where contact force represented in
        grf_truth_local_[0] = -sim_->d_->sensordata[idx_contact_force];
        grf_truth_local_[1] = -sim_->d_->sensordata[idx_contact_force + 1];
        grf_truth_local_[2] = -sim_->d_->sensordata[idx_contact_force + 2];
        mju_rotVecQuat(grf_truth_world_, grf_truth_local_, contact_frame_truth_);

        // publish ground truth data
        pogox_communication::msg::PogoxState ground_truth_msg;

        double sim_time = sim_->d_->time;
        ground_truth_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
        ground_truth_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - ground_truth_msg.header.stamp.sec) * 1e9);
        ground_truth_msg.header.frame_id = "sim_time"; 

        ground_truth_msg.pose.position.x = pos_truth_[0]; 
        ground_truth_msg.pose.position.y = pos_truth_[1]; 
        ground_truth_msg.pose.position.z = pos_truth_[2]; 

        ground_truth_msg.pose.orientation.x = orien_truth_[1];
        ground_truth_msg.pose.orientation.y = orien_truth_[2];
        ground_truth_msg.pose.orientation.z = orien_truth_[3];
        ground_truth_msg.pose.orientation.w = orien_truth_[0];

        ground_truth_msg.twist.linear.x = vel_truth_[0];
        ground_truth_msg.twist.linear.y = vel_truth_[1];
        ground_truth_msg.twist.linear.z = vel_truth_[2];

        ground_truth_msg.twist.angular.x = angular_vel_truth_[0];
        ground_truth_msg.twist.angular.y = angular_vel_truth_[1];
        ground_truth_msg.twist.angular.z = angular_vel_truth_[2];

        ground_truth_msg.accel.linear.x = acc_truth_[0];
        ground_truth_msg.accel.linear.y = acc_truth_[1];
        ground_truth_msg.accel.linear.z = acc_truth_[2];

        ground_truth_msg.accel.angular.x = angular_acc_truth_[0];
        ground_truth_msg.accel.angular.y = angular_acc_truth_[1];
        ground_truth_msg.accel.angular.z = angular_acc_truth_[2];

        ground_truth_msg.leg_joint_state.leg_state[0] = leg_joint_state_truth_[0];
        ground_truth_msg.leg_joint_state.leg_state[1] = leg_joint_state_truth_[1];
        
        ground_truth_msg.contact_state = contact_truth_;
        
        ground_truth_msg.force.contact_force[0] = grf_truth_world_[0];
        ground_truth_msg.force.contact_force[1] = grf_truth_world_[1];
        ground_truth_msg.force.contact_force[2] = grf_truth_world_[2];
        
        // publish the data  
        ground_truth_pub_ptr_->publish(ground_truth_msg);
    }
}


}