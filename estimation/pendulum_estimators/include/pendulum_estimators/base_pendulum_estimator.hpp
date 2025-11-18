#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

namespace pendulum_estimators // estimators are treated as chainable controllers
{ 
    class BasePendulumEstimator : public controller_interface::ChainableControllerInterface
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
    
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
        
        controller_interface::return_type update_reference_from_subscribers() override;    
        
        // sensor readings
        double joint_pos_;
        double joint_vel_;
        double joint_tau_;
        std::array<double, 2> tip_pos_; // y, z
        std::array<double, 2> tip_vel_; // vy, vz

        // ros2_control related
        // command interface names
        std::string joint_name = "pivot";
        std::vector<std::string> joint_interface_types = {
            "position", "velocity", "effort"
        };
        std::string tip_name = "tip_sensor";
        std::vector<std::string> tip_interface_types = {
            "y", "z", "vy", "vz"
        };
    };


} // namespace pendulum_estimators