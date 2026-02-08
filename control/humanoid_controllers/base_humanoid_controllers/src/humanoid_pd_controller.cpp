#include "base_humanoid_controllers/humanoid_pd_controller.hpp"

namespace humanoid_controllers
{

    controller_interface::CallbackReturn HumanoidPDController::on_init()
    {
        controller_interface::CallbackReturn state = BaseHumanoidController::on_init();
        // check whether reference interfaces conflicted
        if (planner_name_ != "" && ref_controller_name_ != "")
        {
            RCLCPP_ERROR(
                this->get_node()->get_logger(),
                "HumanoidPDController::on_init() failed because both planner_name and ref_controller_name are set.");
            return controller_interface::CallbackReturn::ERROR;
        }

        // resize vectors
        joint_pos_ref_.resize(joint_names_.size(), 0.0);
        joint_vel_ref_.resize(joint_names_.size(), 0.0);
        joint_pos_.resize(joint_names_.size(), 0.0);
        joint_vel_.resize(joint_names_.size(), 0.0);
        kp_gains_.resize(joint_names_.size(), 0.0);
        kd_gains_.resize(joint_names_.size(), 0.0);

        // init pd gains and targets from parameters if exist
        joint_pos_ref_ = auto_declare<std::vector<double>>("joint_pos_ref", joint_pos_ref_);
        joint_vel_ref_ = auto_declare<std::vector<double>>("joint_vel_ref", joint_vel_ref_);
        kp_gains_ = auto_declare<std::vector<double>>("kp_gains", kp_gains_);
        kd_gains_ = auto_declare<std::vector<double>>("kd_gains", kd_gains_);

        // initialize PID controllers
        pid_controllers_.init(joint_names_.size());
        pid_controllers_.set_gains(kp_gains_,
            std::vector<double>(joint_names_.size(), 0.0), kd_gains_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration HumanoidPDController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::ALL;
        return config;
    }

    controller_interface::InterfaceConfiguration HumanoidPDController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // state estimation
        if(estimator_name_ != "")
        {
            // Joint interfaces
            for (const auto &joint : joint_names_) {
                for (const auto &iface : joint_state_interface_types_) {
                    config.names.push_back(estimator_name_ + "/" + joint + "_" + iface + "_est");
                }
            }

            // Contact and force wrench sensors
            for (const auto &foot : foot_names_) {
                for (const auto &sensor : foot_sensor_names_) {
                    config.names.push_back(estimator_name_ + "/" + foot + "_" + sensor + "_est");
                }
            }

            // Position (x, y, z)
            config.names.push_back(estimator_name_ + "/" + pos_name_ + "_x_est");
            config.names.push_back(estimator_name_ + "/" + pos_name_ + "_y_est");
            config.names.push_back(estimator_name_ + "/" + pos_name_ + "_z_est");

            // Orientation (w, x, y, z)
            config.names.push_back(estimator_name_ + "/" + ori_name_ + "_w_est");
            config.names.push_back(estimator_name_ + "/" + ori_name_ + "_x_est");
            config.names.push_back(estimator_name_ + "/" + ori_name_ + "_y_est");
            config.names.push_back(estimator_name_ + "/" + ori_name_ + "_z_est");

            // Linear velocity (x, y, z)
            config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_x_est");
            config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_y_est");
            config.names.push_back(estimator_name_ + "/" + lin_vel_name_ + "_z_est");

            // Angular velocity (x, y, z)
            config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_x_est");
            config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_y_est");
            config.names.push_back(estimator_name_ + "/" + ang_vel_name_ + "_z_est");

            // Linear acceleration (x, y, z)
            config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_x_est");
            config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_y_est");
            config.names.push_back(estimator_name_ + "/" + lin_acc_name_ + "_z_est");

            // Angular acceleration (x, y, z)
            config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_x_est");
            config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_y_est");
            config.names.push_back(estimator_name_ + "/" + ang_acc_name_ + "_z_est");
        }


        // reference trajectory
        if(planner_name_ != "")
        {
            for (const auto & joint_name : joint_names_)
            {
                config.names.push_back(planner_name_ + "/" + joint_name + "_position_ref");
                config.names.push_back(planner_name_ + "/" + joint_name + "_velocity_ref");
            }
        }

        return config;
    }

    controller_interface::CallbackReturn HumanoidPDController::on_configure(
        const rclcpp_lifecycle::State & previous_state)
    {
        controller_interface::CallbackReturn state = BaseHumanoidController::on_configure(previous_state);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn HumanoidPDController::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn HumanoidPDController::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type HumanoidPDController::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Filter joint position and velocity from state interfaces
        for (const auto &state_iface : state_interfaces_) {
            const std::string iface_name = state_iface.get_name();
            const std::string prefix_name = state_iface.get_prefix_name();
            const std::string interface_name = state_iface.get_interface_name();
            // Check if this interface is from the estimator
            if (prefix_name == estimator_name_) {
                // Parse joint position interfaces
                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    if (interface_name == joint_names_[i] + "_position_est") {
                        joint_pos_[i] = state_iface.get_value();
                        break;
                    }
                }
                // Parse joint velocity interfaces
                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    if (interface_name == joint_names_[i] + "_velocity_est") {
                        joint_vel_[i] = state_iface.get_value();
                        break;
                    }
                }
            }
        }
        std::vector<double> errors = joint_pos_ref_;
        std::vector<double> errors_dot = joint_vel_ref_;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            errors[i] -= joint_pos_[i];
            errors_dot[i] -= joint_vel_[i];
        }
        // Calculate control commands using PD control law
        std::vector<double> joint_efforts = pid_controllers_.compute(
            errors, errors_dot, period);

        // Get the answers and set command_interfaces_


        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> HumanoidPDController::on_export_reference_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        std::string controller_name = this->get_node()->get_name();
        reference_interfaces_.resize(joint_names_.size() * 2, 0.0);
        // Export position reference interfaces
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            reference_interfaces.emplace_back(
                controller_name,
                joint_names_[i] + "_position_ref",
                &joint_pos_ref_[i]
            );
        }
        // Export velocity reference interfaces
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            reference_interfaces.emplace_back(
                controller_name,
                joint_names_[i] + "_velocity_ref",
                &joint_vel_ref_[i]
            );
        }

        return reference_interfaces;
    }
};
