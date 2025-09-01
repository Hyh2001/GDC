#include "robot_specific/pendulum/pendulum_hardware_interface.hpp"

namespace mujoco_hardware_interface
{
    PendulumMujocoHardwareInterface::PendulumMujocoHardwareInterface()
    : hardware_interface::SystemInterface()
    {
    }

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_configure(const hardware_interface::HardwareInfo & info)
    {
        // build threads for physics and rendering
        if(sim_ptr_){
            stop_flag_ = false;
            renderthreadhandle_ = std::thread([this]() {
                while (!stop_flag_) {
                    if (sim_ptr_) {
                        sim_ptr_->RenderLoop();
                    }
                }
            });
            physicsthreadhandle_ = std::thread([this]() {
                while (!stop_flag_) {
                    if (sim_ptr_) {
                        sim_.PhysicsThread();
                    }
                }
            });
        }
        else{
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }            
    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_shutdown(const hardware_interface::HardwareInfo & info)
    {
        stop_flag_ = true;
        if (renderthreadhandle_.joinable()) {
            renderthreadhandle_.join();
        }
        if (physicsthreadhandle_.joinable()) {
            physicsthreadhandle_.join();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_cleanup(const hardware_interface::HardwareInfo & info)
    {
        stop_flag_ = true;
        if (renderthreadhandle_.joinable()) {
            renderthreadhandle_.join();
        }
        if (physicsthreadhandle_.joinable()) {
            physicsthreadhandle_.join();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_activate(const hardware_interface::HardwareInfo & info)
    {
        // resume physics thread
        stop_flag_ = false;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_deactivate(const hardware_interface::HardwareInfo & info)
    {
        // pause physics thread
        stop_flag_ = true;
        if (renderthreadhandle_.joinable()) {
            renderthreadhandle_.join();
        }
        if (physicsthreadhandle_.joinable()) {
            physicsthreadhandle_.join();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // hardware_interface::return_type PendulumMujocoHardwareInterface::on_error(const hardware_interface::HardwareInfo & info)
    // {
    //     return hardware_interface::return_type::OK;
    // }

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        // get mjcf_path
        auto path = info.hardware_parameters.find("mjcf_path");
        if (path != info.hardware_parameters.end()) {
            mjcf_path_ = path->second;
        } else {
            return hardware_interface::CallbackReturn::ERROR;
        }
        sim_ = MujocoSim(mjcf_path_);
        sim_ptr_ = sim_.getSimPtr();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // std::vector<hardware_interface::StateInterface> PendulumMujocoHardwareInterface::export_state_interfaces()
    // {
    //     std::vector<hardware_interface::StateInterface> state_interfaces;
    //     // TODO: Add state interfaces
    //     return state_interfaces;
    // }

    // std::vector<hardware_interface::CommandInterface> PendulumMujocoHardwareInterface::export_command_interfaces()
    // {
    //     std::vector<hardware_interface::CommandInterface> command_interfaces;
    //     // TODO: Add command interfaces
    //     return command_interfaces;
    // }

    // hardware_interface::return_type PendulumMujocoHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces,
    //                                                                                             const std::vector<std::string> & stop_interfaces)
    // {
    //     // Optional: implement if needed
    //     return hardware_interface::return_type::OK;
    // }

    // hardware_interface::return_type PendulumMujocoHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & start_interfaces,
    //                                                                                             const std::vector<std::string> & stop_interfaces)
    // {
    //     // Optional: implement if needed
    //     return hardware_interface::return_type::OK;
    // }

    hardware_interface::return_type PendulumMujocoHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // TODO: implement reading from hardware
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PendulumMujocoHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // TODO: implement writing to hardware
        return hardware_interface::return_type::OK;
    }

};