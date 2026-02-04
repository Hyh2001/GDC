#include "limx_dynamics_hardware_interfaces/tron1_hardware_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace limx_dynamics_hardware_interfaces{

    Tron1HardwareInterface::Tron1HardwareInterface()
    : BaseHumanoidHardwareInterfaces("tron1_hardware")
    {

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        Tron1HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        // get robot type from parameter
        std::string robot_type_str;
        this->declare_parameter<std::string>("robot_type", "POINT_FOOT");
        this->get_parameter("robot_type", robot_type_str);
        robot_type_ = parse_robot_type(robot_type_str);

        // initialize the sizes of vectors
        if(robot_type_ == FLAT_FOOT || robot_type_ == WHEEL_FOOT){
            joint_positions_.resize(8, 0.0);
            joint_velocities_.resize(8, 0.0);
            joint_efforts_.resize(8, 0.0);
            joint_position_commands_.resize(8, 0.0);
            joint_velocity_commands_.resize(8, 0.0);
            joint_effort_commands_.resize(8, 0.0);
            joint_kp_gains_.resize(8, 0.0);
            joint_kd_gains_.resize(8, 0.0);
            low_state_msg_.motor_state.resize(8);
        } 
        else if(robot_type_ == POINT_FOOT){
            joint_positions_.resize(6, 0.0);
            joint_velocities_.resize(6, 0.0);
            joint_efforts_.resize(6, 0.0);
            joint_position_commands_.resize(6, 0.0);
            joint_velocity_commands_.resize(6, 0.0);
            joint_effort_commands_.resize(6, 0.0);
            joint_kp_gains_.resize(6, 0.0);
            joint_kd_gains_.resize(6, 0.0);
            low_state_msg_.motor_state.resize(6);
        }
        else if(robot_type_ == POINT_FOOT_WITH_ARM){
            joint_positions_.resize(12, 0.0);
            joint_velocities_.resize(12, 0.0);
            joint_efforts_.resize(12, 0.0);
            joint_position_commands_.resize(12, 0.0);
            joint_velocity_commands_.resize(12, 0.0);
            joint_effort_commands_.resize(12, 0.0);
            joint_kp_gains_.resize(12, 0.0);
            joint_kd_gains_.resize(12, 0.0);
            low_state_msg_.motor_state.resize(12);
        }
        else if(robot_type_ == FLAT_FOOT_WITH_ARM || robot_type_ == WHEEL_FOOT_WITH_ARM){
            joint_positions_.resize(14, 0.0);
            joint_velocities_.resize(14, 0.0);
            joint_efforts_.resize(14, 0.0);
            joint_position_commands_.resize(14, 0.0);
            joint_velocity_commands_.resize(14, 0.0);
            joint_effort_commands_.resize(14, 0.0);
            joint_kp_gains_.resize(14, 0.0);
            joint_kd_gains_.resize(14, 0.0);
            low_state_msg_.motor_state.resize(14);
        }

        // subscriber
        auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
        cmd_sub_ptr_ = this->create_subscription<humanoid_msgs::msg::LowCmd>(
            "/tron1/low_cmd", qos, std::bind(&Tron1HardwareInterface::callback_low_cmd, this, std::placeholders::_1));
        // publisher
        low_state_pub_ptr_ = this->create_publisher<humanoid_msgs::msg::LowState>(
            "/tron1/low_state", qos);
        // timer
        timers_.emplace_back(this->create_wall_timer(
            std::chrono::milliseconds(2), std::bind(&Tron1HardwareInterface::publish_low_state, this)));
        
        // API related
        this->declare_parameter<std::string>("robot_ip", "127.0.0.1");
        this->get_parameter("robot_ip", robot_ip_);
        robot_ = limxsdk::PointFoot::getInstance();
        robot_->init(robot_ip_);
        robot_cmd_ = limxsdk::RobotCmd(robot_->getMotorNumber());
        robot_state_ = limxsdk::RobotState(robot_->getMotorNumber());
        if(robot_->getMotorNumber() != joint_positions_.size()){
            RCLCPP_ERROR(
                this->get_logger(),
                "Motor number from robot API (%zu) does not match the initialized size (%zu).",
                static_cast<size_t>(robot_->getMotorNumber()),
                joint_positions_.size());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        // sdk subscribers
        robot_->subscribeRobotState([&](const limxsdk::RobotStateConstPtr &msg)
        {
            mtx_.lock();
            robot_state_ = *msg;
            mtx_.unlock(); });
        robot_->subscribeImuData([&](const limxsdk::ImuDataConstPtr &msg)
        {
            imu_data_ = *msg;
        });
        robot_->subscribeDiagnosticValue([&](const limxsdk::DiagnosticValueConstPtr &msg)
        {
            diagnostic_value_ = *msg;
        });
        // check the diagnostic before activation
        if(!check_hardware()){
            RCLCPP_ERROR(this->get_logger(), "Hardware check failed during configuration.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    bool Tron1HardwareInterface::check_hardware()
    {
        RCLCPP_INFO(this->get_logger(), "Checking hardware diagnostics...");
        
        bool imu_ok = false;
        bool ethercat_ok = false;
        bool calibration_ok = false;
        
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(10); // wait 10 seconds
        
        while (std::chrono::steady_clock::now() - start_time < timeout)
        {
            if (diagnostic_value_.name == "imu" && 
                diagnostic_value_.level == limxsdk::DiagnosticValue::OK && 
                diagnostic_value_.code == 0)
            {
                imu_ok = true;
                RCLCPP_INFO(this->get_logger(), "IMU check passed: %s", diagnostic_value_.message.c_str());
            }
            else if (diagnostic_value_.name == "ethercat" && 
                    diagnostic_value_.level == limxsdk::DiagnosticValue::OK && 
                    diagnostic_value_.code == 0)
            {
                ethercat_ok = true;
                RCLCPP_INFO(this->get_logger(), "EtherCAT check passed: %s", diagnostic_value_.message.c_str());
            }
            else if (diagnostic_value_.name == "calibration" && 
                    diagnostic_value_.level == limxsdk::DiagnosticValue::OK && 
                    diagnostic_value_.code == 0)
            {
                calibration_ok = true;
                RCLCPP_INFO(this->get_logger(), "Calibration check passed: %s", diagnostic_value_.message.c_str());
            }
            
            if (imu_ok && ethercat_ok && calibration_ok)
            {
                RCLCPP_INFO(this->get_logger(), "All hardware diagnostics passed!");
                return true;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // timeout
        RCLCPP_ERROR(this->get_logger(), "Hardware diagnostic check timeout!");
        if (!imu_ok) RCLCPP_ERROR(this->get_logger(), "IMU diagnostic failed");
        if (!ethercat_ok) RCLCPP_ERROR(this->get_logger(), "EtherCAT diagnostic failed");
        if (!calibration_ok) RCLCPP_ERROR(this->get_logger(), "Calibration diagnostic failed");
        
        return false;
    }

    void Tron1HardwareInterface::reset()
    {
        // reset all the params
        // sensor readings
        joint_positions_.assign(joint_positions_.size(), 0.0);
        joint_velocities_.assign(joint_velocities_.size(), 0.0);
        joint_efforts_.assign(joint_efforts_.size(), 0.0);

        gyro_.fill(0.0);
        accelerometer_.fill(0.0);
        contacts_.fill(false);

        // motor commands
        joint_position_commands_.assign(joint_position_commands_.size(), 0.0);
        joint_velocity_commands_.assign(joint_velocity_commands_.size(), 0.0);
        joint_effort_commands_.assign(joint_effort_commands_.size(), 0.0);
        joint_kp_gains_.assign(joint_kp_gains_.size(), 0.0);
        joint_kd_gains_.assign(joint_kd_gains_.size(), 0.0);
    }

    void Tron1HardwareInterface::read()
    {
        // publish sensor readings
        // joint order is L -> R
        for (size_t  i=0; i < robot_state_.q.size(); ++i){
            joint_positions_[i] = robot_state_.q[i];
            joint_velocities_[i] = robot_state_.dq[i];
            joint_efforts_[i] = robot_state_.tau[i];
        }
        // imu
        gyro_[0] = imu_data_.gyro[0];
        gyro_[1] = imu_data_.gyro[1];
        gyro_[2] = imu_data_.gyro[2];
        accelerometer_[0] = imu_data_.acc[0];
        accelerometer_[1] = imu_data_.acc[1];
        accelerometer_[2] = imu_data_.acc[2];
        quat_[0] = imu_data_.quat[0];
        quat_[1] = imu_data_.quat[1];
        quat_[2] = imu_data_.quat[2];
        quat_[3] = imu_data_.quat[3];

        // publish
        low_state_msg_.header.stamp = this->now();
        for (size_t i = 0; i < joint_positions_.size(); ++i) {
            low_state_msg_.motor_state[i].q = joint_positions_[i];
            low_state_msg_.motor_state[i].dq = joint_velocities_[i];
            low_state_msg_.motor_state[i].tau = joint_efforts_[i];
        }
        low_state_msg_.imu.angular_velocity.x = gyro_[0];
        low_state_msg_.imu.angular_velocity.y = gyro_[1];
        low_state_msg_.imu.angular_velocity.z = gyro_[2];
        low_state_msg_.imu.linear_acceleration.x = accelerometer_[0];
        low_state_msg_.imu.linear_acceleration.y = accelerometer_[1];
        low_state_msg_.imu.linear_acceleration.z = accelerometer_[2];
        low_state_msg_.imu.orientation.w = quat_[0];
        low_state_msg_.imu.orientation.x = quat_[1];
        low_state_msg_.imu.orientation.y = quat_[2];
        low_state_msg_.imu.orientation.z = quat_[3];
        
        low_state_pub_ptr_->publish(low_state_msg_);
    }

    void Tron1HardwareInterface::callback_low_cmd(const humanoid_msgs::msg::LowCmd::SharedPtr msg)
    {
        // receive motor commands
        for (size_t i = 0; i < joint_position_commands_.size(); ++i){
            joint_position_commands_[i] = msg->motor_cmd[i].q;
            joint_velocity_commands_[i] = msg->motor_cmd[i].dq;
            joint_effort_commands_[i] = msg->motor_cmd[i].tau;
            joint_kp_gains_[i] = msg->motor_cmd[i].kp;
            joint_kd_gains_[i] = msg->motor_cmd[i].kd;
        }
        // send to robot
        write();
    }

    void Tron1HardwareInterface::publish_low_state()
    {
        read();
    }

    void Tron1HardwareInterface::write()
    {
        // send motor commands
        for (size_t i = 0; i < robot_cmd_.q.size(); ++i){
            robot_cmd_.q[i] = joint_position_commands_[i];
            robot_cmd_.dq[i] = joint_velocity_commands_[i];
            robot_cmd_.tau[i] = joint_effort_commands_[i];
            robot_cmd_.Kp[i] = joint_kp_gains_[i];
            robot_cmd_.Kd[i] = joint_kd_gains_[i];
        }
        robot_->publishRobotCmd(robot_cmd_);
    }

    Tron1HardwareInterface::RobotType Tron1HardwareInterface::parse_robot_type(const std::string& robot_type_str)
    {
        if (robot_type_str == "POINT_FOOT" || robot_type_str == "point_foot") {
            return POINT_FOOT;
        } else if (robot_type_str == "FLAT_FOOT" || robot_type_str == "flat_foot") {
            return FLAT_FOOT;
        } else if (robot_type_str == "WHEEL_FOOT" || robot_type_str == "wheel_foot") {
            return WHEEL_FOOT;
        } else if (robot_type_str == "POINT_FOOT_WITH_ARM" || robot_type_str == "point_foot_with_arm") {
            return POINT_FOOT_WITH_ARM;
        } else if (robot_type_str == "FLAT_FOOT_WITH_ARM" || robot_type_str == "flat_foot_with_arm") {
            return FLAT_FOOT_WITH_ARM;
        } else if (robot_type_str == "WHEEL_FOOT_WITH_ARM" || robot_type_str == "wheel_foot_with_arm") {
            return WHEEL_FOOT_WITH_ARM;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown robot type string: %s. Defaulting to POINT_FOOT.", robot_type_str.c_str());
            throw std::runtime_error("Unknown robot type string.");
        }
    }

}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<limx_dynamics_hardware_interfaces::Tron1HardwareInterface>();
  const auto configured_state = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configured_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to transition tron1 hardware interface node to 'configured'.");
    rclcpp::shutdown();
    return 1;
  }

  const auto activated_state = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activated_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to transition tron1 hardware interface node to 'active'.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
