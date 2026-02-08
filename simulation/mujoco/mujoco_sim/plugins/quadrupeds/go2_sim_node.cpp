#include "quadrupeds/go2_sim_node.hpp"

namespace mujoco_sim
{

Go2SimNode::Go2SimNode() : MujocoSimNodeBase("go2_sim")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  cmd_sub_ptr_ = this->create_subscription<quadruped_msgs::msg::LowCmd>(
      "/go2/low_cmd", qos, std::bind(&Go2SimNode::callback_low_cmd, this, std::placeholders::_1));

  low_state_pub_ptr_ = this->create_publisher<quadruped_msgs::msg::LowState>("/go2/low_state", qos);

  timers_.emplace_back(this->create_wall_timer(2ms, std::bind(&Go2SimNode::callback_low_state, this)));

  reset_params();
}

void Go2SimNode::reset_params()
{
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

void Go2SimNode::callback_low_state()
{
  // get the sensor data
  if (sim_->d_)
  {
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

    for (int i = 0; i < 12; i++)
    {
      low_state_msg.motor_state[i].q = sim_->d_->sensordata[idx_joint_pos + i];
      low_state_msg.motor_state[i].dq = sim_->d_->sensordata[idx_joint_vel + i];
      low_state_msg.motor_state[i].tau = sim_->d_->sensordata[idx_joint_torque + i];
    }
    bool contact;
    for (int i = 0; i < 4; i++)
    {
      if (sim_->d_->sensordata[idx_contact + i] > 0.0)
      {
        contact = true;
      }
      else
      {
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

void Go2SimNode::callback_low_cmd(const quadruped_msgs::msg::LowCmd::SharedPtr msg)
{
  if (sim_->d_)
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    // apply the motor commands
    // assume actuator orders of position -> velocity -> torque
    for (size_t i = 0; i < joint_pos_.size(); i++)
    {
      switch (msg->motor_cmd[i].mode)
      {
        case uint8_t(0):  // no control
          // sim_->d_->ctrl[i] = 0.0;
          break;
        case uint8_t(1):                                                       // position control
          sim_->m_->actuator_gainprm[i * mjNGAIN + 0] = msg->motor_cmd[i].kp;  // set kp
          sim_->m_->actuator_biasprm[i * mjNBIAS + 1] = -msg->motor_cmd[i].kp;
          sim_->m_->actuator_biasprm[i * mjNBIAS + 2] = -msg->motor_cmd[i].kd;  // set kd
          sim_->d_->ctrl[i] = msg->motor_cmd[i].q;
          break;
        case uint8_t(2):  // velocity control
          sim_->m_->actuator_gainprm[(joint_pos_.size() + i) * mjNGAIN + 0] = msg->motor_cmd[i].kd;
          sim_->m_->actuator_biasprm[(joint_pos_.size() + i) * mjNBIAS + 2] = -msg->motor_cmd[i].kd;
          sim_->d_->ctrl[i + joint_pos_.size()] = msg->motor_cmd[i].dq;
          break;
        case uint8_t(3):  // torque control
          sim_->d_->ctrl[i + 2 * joint_pos_.size()] = msg->motor_cmd[i].tau;
          break;
        case uint8_t(4):                                                       // torque + pd
          sim_->m_->actuator_gainprm[i * mjNGAIN + 0] = msg->motor_cmd[i].kp;  // set kp
          sim_->m_->actuator_biasprm[i * mjNBIAS + 1] = -msg->motor_cmd[i].kp;
          sim_->m_->actuator_biasprm[i * mjNBIAS + 2] = -msg->motor_cmd[i].kd;  // set kd
          sim_->m_->actuator_gainprm[(joint_pos_.size() + i) * mjNGAIN + 0] = msg->motor_cmd[i].kd;
          sim_->m_->actuator_biasprm[(joint_pos_.size() + i) * mjNBIAS + 2] = 0.0;
          sim_->d_->ctrl[i] = msg->motor_cmd[i].q;
          sim_->d_->ctrl[i + joint_pos_.size()] = msg->motor_cmd[i].dq;
          sim_->d_->ctrl[i + 2 * joint_pos_.size()] = msg->motor_cmd[i].tau;
          break;
        case uint8_t(5):  // actuator network
          RCLCPP_ERROR(this->get_logger(), "Actuator network mode not implemented yet.");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown control mode: %d for joint %zu", msg->motor_cmd[i].mode, i);
          break;
      }
    }
  }
}

Go2SimGroundTruth::Go2SimGroundTruth() : Go2SimNode()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);

  ground_truth_pub_ptr_ = this->create_publisher<quadruped_msgs::msg::QuadEst>("/go2/quad_est", qos);

  timers_.emplace_back(this->create_wall_timer(2ms, std::bind(&Go2SimGroundTruth::ground_truth_callback, this)));

  reset_params();
}

void Go2SimGroundTruth::reset_params()
{
  // reset all the params
  Go2SimNode::reset_params();
  // ground truth
  pos_truth_ = {0.0, 0.0, 0.0};        // x, y, z
  ori_truth_ = {1.0, 0.0, 0.0, 0.0};   // w, x, y, z
  lin_vel_truth_ = {0.0, 0.0, 0.0};    // vx, vy, vz
  lin_accel_truth_ = {0.0, 0.0, 0.0};  // ax, ay, az
  ang_vel_truth_ = {0.0, 0.0, 0.0};    // wx, wy, wz
  ang_acc_truth_ = {0.0, 0.0, 0.0};    // alphax, alphay, alphaz
}

void Go2SimGroundTruth::ground_truth_callback()
{
  if (sim_->d_)
  {
    quadruped_msgs::msg::QuadEst quad_est_msg = quadruped_msgs::msg::QuadEst();

    // lock the thread
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    const int idx_pos_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_pos")];
    const int idx_orien_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "Frame_quat")];
    const int idx_linvel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_linvel")];
    const int idx_angvel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_angvel")];
    const int idx_linaccel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_linacc")];
    const int idx_angaccel_truth = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "frame_angacc")];
    const int idx_joint_pos = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_pos")];
    const int idx_joint_vel = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_vel")];
    const int idx_joint_torque = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_hip_torque")];
    const int idx_contact = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_contact_sensor")];
    const int idx_contact_force = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_grf")];
    const int idx_contact_orient = sim_->m_->sensor_adr[mj_name2id(sim_->m_, mjOBJ_SENSOR, "FR_foot_quat")];

    // position
    quad_est_msg.pose.position.x = sim_->d_->sensordata[idx_pos_truth + 0];
    quad_est_msg.pose.position.y = sim_->d_->sensordata[idx_pos_truth + 1];
    quad_est_msg.pose.position.z = sim_->d_->sensordata[idx_pos_truth + 2];
    // orientation
    quad_est_msg.pose.orientation.w = sim_->d_->sensordata[idx_orien_truth + 0];
    quad_est_msg.pose.orientation.x = sim_->d_->sensordata[idx_orien_truth + 1];
    quad_est_msg.pose.orientation.y = sim_->d_->sensordata[idx_orien_truth + 2];
    quad_est_msg.pose.orientation.z = sim_->d_->sensordata[idx_orien_truth + 3];
    // linear velocity
    quad_est_msg.twist.linear.x = sim_->d_->sensordata[idx_linvel_truth + 0];
    quad_est_msg.twist.linear.y = sim_->d_->sensordata[idx_linvel_truth + 1];
    quad_est_msg.twist.linear.z = sim_->d_->sensordata[idx_linvel_truth + 2];
    // angular velocity
    quad_est_msg.twist.angular.x = sim_->d_->sensordata[idx_angvel_truth + 0];
    quad_est_msg.twist.angular.y = sim_->d_->sensordata[idx_angvel_truth + 1];
    quad_est_msg.twist.angular.z = sim_->d_->sensordata[idx_angvel_truth + 2];
    // linear acceleration
    quad_est_msg.accel.linear.x = sim_->d_->sensordata[idx_linaccel_truth + 0];
    quad_est_msg.accel.linear.y = sim_->d_->sensordata[idx_linaccel_truth + 1];
    quad_est_msg.accel.linear.z = sim_->d_->sensordata[idx_linaccel_truth + 2];
    // angular acceleration
    quad_est_msg.accel.angular.x = sim_->d_->sensordata[idx_angaccel_truth + 0];
    quad_est_msg.accel.angular.y = sim_->d_->sensordata[idx_angaccel_truth + 1];
    quad_est_msg.accel.angular.z = sim_->d_->sensordata[idx_angaccel_truth + 2];

    // motor states
    for (int i = 0; i < 12; i++)
    {
      quad_est_msg.motor_state[i].q = sim_->d_->sensordata[idx_joint_pos + i];
      quad_est_msg.motor_state[i].dq = sim_->d_->sensordata[idx_joint_vel + i];
      quad_est_msg.motor_state[i].tau = sim_->d_->sensordata[idx_joint_torque + i];
    }
    // contact states
    bool contact;
    for (int i = 0; i < 4; i++)
    {
      if (sim_->d_->sensordata[idx_contact + i] > 0.0)
      {
        contact = true;
      }
      else
      {
        contact = false;
      }
      quad_est_msg.contact_state[i].contact = contact;
    }
    // contact forces
    mjtNum foot_quat[4][4];
    for (int foot = 0; foot < 4; foot++)
    {
      foot_quat[foot][0] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 0];
      foot_quat[foot][1] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 1];
      foot_quat[foot][2] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 2];
      foot_quat[foot][3] = sim_->d_->sensordata[idx_contact_orient + foot * 4 + 3];
    }
    mjtNum foot_force_site[4][3];
    mjtNum foot_force_world[4][3];
    for (int foot = 0; foot < 4; foot++)
    {
      // NOTE1: grf sensor senses the force exerted on the child body by the parent body
      // when the child body is almost massless, it = -grf
      foot_force_site[foot][0] = -sim_->d_->sensordata[idx_contact_force + foot * 3 + 0];
      foot_force_site[foot][1] = -sim_->d_->sensordata[idx_contact_force + foot * 3 + 1];
      foot_force_site[foot][2] = -sim_->d_->sensordata[idx_contact_force + foot * 3 + 2];
      mju_rotVecQuat(foot_force_world[foot], foot_force_site[foot], foot_quat[foot]);
      quad_est_msg.contact_force[foot].force.x = foot_force_world[foot][0];
      quad_est_msg.contact_force[foot].force.y = foot_force_world[foot][1];
      quad_est_msg.contact_force[foot].force.z = foot_force_world[foot][2];
    }
    double sim_time = sim_->d_->time;
    quad_est_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
    quad_est_msg.header.stamp.nanosec = static_cast<uint32_t>((sim_time - quad_est_msg.header.stamp.sec) * 1e9);
    quad_est_msg.header.frame_id = "sim_time";

    ground_truth_pub_ptr_->publish(quad_est_msg);
  }
}

}  // namespace mujoco_sim

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_sim::Go2SimNode, mujoco_sim::MujocoSimNodeBase)
PLUGINLIB_EXPORT_CLASS(mujoco_sim::Go2SimGroundTruth, mujoco_sim::MujocoSimNodeBase)
