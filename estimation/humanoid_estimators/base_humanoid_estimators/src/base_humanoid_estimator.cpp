#include "base_humanoid_estimators/base_humanoid_estimator.hpp"

namespace humanoid_estimators
{

    controller_interface::CallbackReturn BaseHumanoidEstimator::on_init()
    {
        // interface names for parameters
        joint_names_ = auto_declare<std::vector<std::string>>("joint_names", joint_names_);
        num_joints_ = joint_names_.size();
        joint_state_interface_types_ = auto_declare<std::vector<std::string>>(
            "joint_state_interfaces", joint_state_interface_types_);
        joint_command_interface_types_ = auto_declare<std::vector<std::string>>(
            "joint_command_interfaces", joint_command_interface_types_);
        foot_names_ = auto_declare<std::vector<std::string>>(
            "foot_names", foot_names_);
        foot_sensor_names_ = auto_declare<std::vector<std::string>>(
            "foot_sensor_names", foot_sensor_names_);
        pos_name_ = auto_declare<std::string>("pos_name", pos_name_);
        ori_name_ =  auto_declare<std::string>("ori_name", ori_name_);
        lin_vel_name_ = auto_declare<std::string>("lin_vel_name", lin_vel_name_);
        ang_vel_name_ = auto_declare<std::string>("ang_vel_name", ang_vel_name_);
        lin_acc_name_ = auto_declare<std::string>("lin_acc_name", lin_acc_name_);
        ang_acc_name_ = auto_declare<std::string>("ang_acc_name", ang_acc_name_);
        
        joint_pos_.resize(num_joints_, 0.0);
        joint_vel_.resize(num_joints_, 0.0);
        joint_acc_.resize(num_joints_, 0.0);
        joint_tau_.resize(num_joints_, 0.0);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration BaseHumanoidEstimator::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }

    controller_interface::InterfaceConfiguration BaseHumanoidEstimator::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::ALL;
        return config;
    }

    controller_interface::CallbackReturn BaseHumanoidEstimator::on_configure(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BaseHumanoidEstimator::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BaseHumanoidEstimator::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> BaseHumanoidEstimator::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        std::string estimator_name = this->get_name(); //
        size_t joint_idx = 0;
        // Joint interfaces
        for (const auto &joint : joint_names_) {
            for (const auto &iface : joint_state_interface_types_) {
                if (iface == "position") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, joint + "_" + iface + "_est", &joint_pos_[joint_idx]));
                } else if (iface == "velocity") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, joint + "_" + iface + "_est", &joint_vel_[joint_idx]));
                } else if (iface == "effort") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, joint + "_" + iface + "_est", &joint_tau_[joint_idx]));
                }
            }
            joint_idx++;
        }

        // Contact and force wrench sensors
        size_t foot_idx = 0;
        for (const auto &foot : foot_names_) {
            for (const auto &sensor : foot_sensor_names_) {
                if (sensor == "state") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_states_[foot_idx]));
                } else if (sensor == "force_x") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_wrenches_[foot_idx][0]));
                } else if (sensor == "force_y") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_wrenches_[foot_idx][1]));
                } else if (sensor == "force_z") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_wrenches_[foot_idx][2]));
                } else if (sensor == "torque_x") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_wrenches_[foot_idx][3]));
                } else if (sensor == "torque_y") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_wrenches_[foot_idx][4]));
                } else if (sensor == "torque_z") {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(
                        estimator_name, foot + "_" + sensor + "_est", &contact_wrenches_[foot_idx][5]));
                }
            }
            foot_idx++;
        }

        // position
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, pos_name_ + "_x" + "_est", &pos_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, pos_name_ + "_y" + "_est", &pos_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, pos_name_ + "_z" + "_est", &pos_[2]));
    
        // Orientation
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name_ + "_w" + "_est", &ori_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name_ + "_x" + "_est", &ori_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name_ + "_y" + "_est", &ori_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ori_name_ + "_z" + "_est", &ori_[3]));
            
        // Global Linear velocity
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_vel_name_ + "_x" + "_est", &lin_vel_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_vel_name_ + "_y" + "_est", &lin_vel_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_vel_name_ + "_z" + "_est", &lin_vel_[2]));

        // Global Angular velocity
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_vel_name_ + "_x" + "_est", &ang_vel_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_vel_name_ + "_y" + "_est", &ang_vel_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_vel_name_ + "_z" + "_est", &ang_vel_[2]));

        // Global Linear acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_acc_name_ + "_x" + "_est", &lin_acc_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_acc_name_ + "_y" + "_est", &lin_acc_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, lin_acc_name_ + "_z" + "_est", &lin_acc_[2]));
        
        // Global Angular acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_acc_name_ + "_x" + "_est", &ang_acc_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_acc_name_ + "_y" + "_est", &ang_acc_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            estimator_name, ang_acc_name_ + "_z" + "_est", &ang_acc_[2]));
    
        return state_interfaces;
    }

    controller_interface::return_type BaseHumanoidEstimator::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        return controller_interface::return_type::OK;
    }
}; 