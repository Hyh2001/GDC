#include "base_hardware_interfaces/base_humanoid_hardware_interface.hpp"
#include "limxsdk/pointfoot.h"


namespace limx_dynamics_hardware_interfaces{

    class Tron1HardwareInterface : public base_hardware_interfaces::BaseHumanoidHardwareInterfaces
    {
    public:
        Tron1HardwareInterface();

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_configure(const rclcpp_lifecycle::State & previous_state) override;

        bool check_hardware(); 

        void reset() override;
        void read() override;
        void write() override;

    protected:
        // robot_type
        enum RobotType{
            POINT_FOOT = 0,
            FLAT_FOOT = 1,
            WHEEL_FOOT = 2,
            POINT_FOOT_WITH_ARM = 3,
            FLAT_FOOT_WITH_ARM = 4,
            WHEEL_FOOT_WITH_ARM = 5
        } robot_type_;
        RobotType parse_robot_type(const std::string& robot_type_str);
        
        void callback_low_cmd(const humanoid_msgs::msg::LowCmd::SharedPtr msg);
        void publish_low_state(); 
        
        humanoid_msgs::msg::LowState low_state_msg_;

        // api
        std::mutex mtx_;
        std::string robot_ip_; 
        limxsdk::PointFoot* robot_;
        limxsdk::RobotCmd robot_cmd_;
        limxsdk::RobotState robot_state_;
        limxsdk::ImuData imu_data_;
        limxsdk::DiagnosticValue diagnostic_value_;
        std::atomic<bool> imu_diagnostic_received_{false};
        std::atomic<bool> ethercat_diagnostic_received_{false};
        std::atomic<bool> calibration_diagnostic_received_{false};
    };


} // namespace limx_dynamics_hardware_interfaces