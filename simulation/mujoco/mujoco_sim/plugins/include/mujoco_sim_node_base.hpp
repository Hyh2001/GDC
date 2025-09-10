#ifndef MUJOCO_SIM_NODE_BASE_HPP
#define MUJOCO_SIM_NODE_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include "mujoco_sim.h"

namespace mujoco_sim{

class MujocoSimNodeBase: public rclcpp::Node {
public:
    MujocoSimNodeBase(const std::string& node_name)
        : Node(node_name){        
    }
    virtual ~MujocoSimNodeBase() = default;

    void set_sim_ptr(mj::Simulate* sim) { sim_ = sim; }

protected:
    mj::Simulate* sim_;

}; 

} // namespace mujoco_sim

#endif