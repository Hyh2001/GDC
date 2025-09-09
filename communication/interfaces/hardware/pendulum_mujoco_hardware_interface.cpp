#include "include/pendulum_mujoco_hardware_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace mujoco_hardware_interface
{
    PendulumMujocoHardwareInterface::PendulumMujocoHardwareInterface()
    : hardware_interface::SystemInterface()
    {
    }

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
    {   
        // // build threads for physics and rendering
        // if(sim_ptr_){
        //     stop_flag_ = false;
        //     // renderthreadhandle_ = std::thread([this]() {
        //     //     while (!stop_flag_) {
        //     //         if (sim_ptr_) {
        //     //             sim_ptr_->RenderLoop();
        //     //         }
        //     //     }
        //     // });
        //     renderthreadhandle_ = std::thread([this]() {
        //         if (sim_ptr_) {
        //             sim_ptr_->RenderLoop();
        //         }
        //     });
        //     physicsthreadhandle_ = std::thread([this]() {
        //         // while (!stop_flag_) {
        //         //     if (sim_ptr_) {
        //         //         sim_->PhysicsThread();
        //         //     }
        //         // }
        //         if(!stop_flag_) {
        //             if (sim_ptr_) {
        //                 sim_->PhysicsThread();
        //             }   
        //         }
        //     });
        // }
        // else{
        //     return hardware_interface::CallbackReturn::ERROR;
        // }
        return hardware_interface::CallbackReturn::SUCCESS;
    }        

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
    {
        stop_flag_ = true;
        // if (renderthreadhandle_.joinable()) {
        //     renderthreadhandle_.join();
        // }
        if (physicsthreadhandle_.joinable()) {
            physicsthreadhandle_.join();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        stop_flag_ = true;
        // if (renderthreadhandle_.joinable()) {
        //     renderthreadhandle_.join();
        // }
        if (physicsthreadhandle_.joinable()) {
            physicsthreadhandle_.join();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        // resume physics thread
        stop_flag_ = false;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PendulumMujocoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        // pause physics thread
        stop_flag_ = true;
        // if (renderthreadhandle_.joinable()) {
        //     renderthreadhandle_.join();
        // }
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
        std::string param_value;
        if (path != info.hardware_parameters.end()) {
            param_value = path->second;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "mjcf_path not specified.");
            return hardware_interface::CallbackReturn::ERROR;
        }

         // Split into package and relative path
        auto delim_pos = param_value.find('/');
        if (delim_pos == std::string::npos) {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Invalid mjcf_path format: '%s'", param_value.c_str());
            return CallbackReturn::ERROR;
        }

        std::string package_name = param_value.substr(0, delim_pos);
        std::string relative_path = param_value.substr(delim_pos + 1);

        std::string package_share_path;
        try {
            package_share_path = ament_index_cpp::get_package_share_directory(package_name);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Package '%s' not found: %s", package_name.c_str(), e.what());
            return CallbackReturn::ERROR;
        }

        mjcf_path_ = package_share_path + "/" + relative_path;
        sim_ = std::make_unique<MujocoSim>(mjcf_path_);
        sim_ptr_ = sim_->getSimPtr();
        std::cout << "hardware interface initialized with mjcf path: " << mjcf_path_ << std::endl;  
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PendulumMujocoHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back("pivot", "position", &pivot_position_);
        state_interfaces.emplace_back("pivot", "velocity", &pivot_velocity_);
        state_interfaces.emplace_back("pivot", "effort", &pivot_effort_);
        state_interfaces.emplace_back("tip_sensor", "y", &tip_sensor_y_);
        state_interfaces.emplace_back("tip_sensor", "z", &tip_sensor_z_);
        state_interfaces.emplace_back("tip_sensor", "vy", &tip_sensor_vy_);
        state_interfaces.emplace_back("tip_sensor", "vz", &tip_sensor_vz_);
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> PendulumMujocoHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back("pivot", "effort", &pivot_effort_command_);
        return command_interfaces;
    }

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

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_hardware_interface::PendulumMujocoHardwareInterface, 
            hardware_interface::SystemInterface)