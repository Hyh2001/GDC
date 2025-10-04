#include "base_quadruped_controllers/edamp.hpp"

namespace quadruped_controllers
{

    controller_interface::CallbackReturn Edamp::on_init()
    {
        // joint command interfaces
        // for (size_t i = 0; i < joint_names_.size(); ++i) {
        //     for (size_t j = 0; j < joint_command_interface_types_.size(); ++j) {
        //         command_interface_names_.emplace_back(joint_names_[i] + "/" + joint_command_interface_types_[j]);
        //     }
        // }
        // // reference command interfaces
        // for (size_t i = 0; i < joint_names_.size(); ++i) {
        //     for (size_t j = 0; j < joint_state_interface_types_.size(); ++j) {
        //         reference_interface_names_.emplace_back(joint_names_[i] + "/" + joint_state_interface_types_[j]);
        //     }
        // }

        return controller_interface::CallbackReturn::SUCCESS;
    }


};
