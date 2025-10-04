

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

namespace quadruped_controllers // estimator as a chainable controller
{
    class BaseEstimator : public controller_interface::ChainableControllerInterface
    {
    public:
        controller_interface::CallbackReturn on_init() override;
    
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        bool on_set_chained_mode(bool chained_mode) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    protected:
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
        
        controller_interface::return_type update_reference_from_subscribers() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        std::vector<std::string> reference_interface_names_;
        std::vector<std::string> command_interface_names_;
    };


}; 