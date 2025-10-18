#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

namespace quadruped_controllers // estimators are treated as chainable controllers
{ 
    class BaseQuadrupedEstimator : public controller_interface::ChainableControllerInterface
    { 
    public:
        controller_interface::CallbackReturn on_init() override;
    
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;


    protected:
        bool on_set_chained_mode(bool chained_mode) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
        // FR FL RR RL
        std::array<double, 12> joint_pos_;
        std::array<double, 12> joint_vel_;
        std::array<double, 12> joint_acc_;
        std::array<double, 12> joint_tau_;
        std::array<double, 4> contact_states_;
        std::array<double, 12> contact_forces_;
        std::array<double, 3> pos_;
        std::array<double, 4> orientation_; // quaternion, w, x, y, z 
        std::array<double, 3> lin_vel_;
        std::array<double, 3> ang_vel_;
        std::array<double, 3> lin_acc_;
        std::array<double, 3> ang_acc_;

        // ros2_control related
        // command interface names
        std::vector<std::string> joint_names = {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        };
        std::vector<std::string> joint_interface_types = {
            "position", "velocity", "acceleration", "effort"
        };
        std::vector<std::string> foot_names = {
            "FR_foot", "FL_foot", "RR_foot", "RL_foot"
        };
        std::vector<std::string> foot_sensor_names = {
            "state", "force_x", "force_y", "force_z"
        };
        std::string pos_name = "global_pos";
        std::string ori_name = "orientation"; // wxyz
        std::string lin_vel_name = "global_lin_vel";
        std::string ang_vel_name = "global_ang_vel";
        std::string lin_acc_name = "global_lin_acc";
        std::string ang_acc_name = "global_ang_acc";

    };


} // namespace quadruped_controllers