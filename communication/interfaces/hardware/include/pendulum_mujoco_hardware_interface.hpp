#ifndef PENDULUM_HARDWARE_INTERFACE_HPP
#define PENDULUM_HARDWARE_INTERFACE_HPP

#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "mujoco_sim.h"

namespace mujoco_hardware_interface
{
    using namespace mujoco_sim;
    class PendulumMujocoHardwareInterface : public hardware_interface::SystemInterface
    {
    public: 
        PendulumMujocoHardwareInterface();

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        // hardware_interface::CallbackReturn on_error(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        // hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces,
        //                                                            const std::vector<std::string> & stop_interfaces) override;
        // hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces,
        //                                                            const std::vector<std::string> & stop_interfaces) override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    protected:
        // simulation related
        std::string mjcf_path_;
        std::unique_ptr<MujocoSim> sim_;
        mj::Simulate* sim_ptr_;
        std::thread physicsthreadhandle_;
        std::thread renderthreadhandle_;
        std::atomic<bool> stop_flag_{false};

        double pivot_position_ = 0.0;
        double pivot_velocity_ = 0.0;
        double pivot_effort_ = 0.0;
        double tip_sensor_y_ = 0.0;
        double tip_sensor_z_ = 0.0;
        double tip_sensor_vy_ = 0.0;
        double tip_sensor_vz_ = 0.0;
        double pivot_effort_command_ = 0.0;
    }; 

}; 


#endif // PENDULUM_MUJOCO_HARDWARE_INTERFACE_HPP