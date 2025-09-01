#ifndef PENDULUM_HARDWARE_INTERFACE_HPP
#define PENDULUM_HARDWARE_INTERFACE_HPP

#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "mujoco_sim.h"

namespace mujoco_hardware_interface
{
    using namespace mujoco_sim;
    class PendulumMujocoHardwareInterface : public hardware_interface::SystemInterface
    {
    public: 
        PendulumMujocoHardwareInterface();

        hardware_interface::CallbackReturn on_configure(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_cleanup(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_shutdown(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_activate(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_deactivate(const hardware_interface::HardwareInfo & info) override;
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
        MujocoSim sim_;
        mj::Simulate* sim_ptr_;
        std::thread physicsthreadhandle_;
        std::thread renderthreadhandle_;
        std::atomic<bool> stop_flag_{false};
    
    }; 

}; 


#endif // PENDULUM_MUJOCO_HARDWARE_INTERFACE_HPP