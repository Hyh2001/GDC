#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "controller_interface/chainable_controller_interface.hpp"

namespace base_planners
{
/* 
    VelocityPlanner implements a velocity planner taking velocity commands and 
    applying the command. 
*/

class VelocityPlanner : public controller_interface::ChainableControllerInterface
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

    // reference
    std::string ref_planner_=""; // upper level planner

    std::array<double, 3> max_velocity_{1.0, 1.0, 0.5}; // vx, vy, yaw rate    
    std::array<double, 3> velocity_cmd_{0.0, 0.0, 0.0}; // vx, vy, yaw rate
};




}; // base_planners