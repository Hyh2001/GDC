#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "Eigen/Dense"

#include "controller_interface/chainable_controller_interface.hpp"
#include "control_toolbox/low_pass_filter.hpp"

namespace base_planners
{
/* 
    JoystickVelocityPlanner implements a velocity planner taking velocity commands from joystick signals and 
    applying a low-pass filter to smooth the commands.
*/

class JoystickVelocityPlanner : public controller_interface::ChainableControllerInterface
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
    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time & time, const rclcpp::Duration & period) override; 

    std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

    double sampling_frequency_{50.0}; // Hz
    double damping_frequency_{1.0}; // Hz
    double damping_intensity_{0.0}; // dB
    std::array<double, 3> max_velocity_{1.0, 1.0, 0.5}; // vx, vy, yaw rate
    std::array<double, 3> velocity_cmd_raw_{0.0, 0.0, 0.0}; // vx, vy, yaw rate    
    std::array<double, 3> velocity_cmd_{0.0, 0.0, 0.0}; // vx, vy, yaw rate
    rclcpp::Node::SharedPtr node_ptr_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_ = nullptr;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::array<std::shared_ptr<control_toolbox::LowPassFilter<double>>, 3> lp_filters_;
};




}; // base_planners