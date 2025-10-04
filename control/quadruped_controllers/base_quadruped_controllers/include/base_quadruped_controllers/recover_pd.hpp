#pragma once

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

namespace quadruped_controllers
{

    class RecoverPD : public controller_interface::ChainableControllerInterface
    {
    };
};