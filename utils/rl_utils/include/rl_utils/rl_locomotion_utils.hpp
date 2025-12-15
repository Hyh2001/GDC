#include <vector>

#include "Eigen/Dense"

namespace rl_utils {

    void projected_gravity(const Eigen::Quaterniond& base_w,
                        Eigen::Vector3d& gravity_proj) 
    {
        /* 
            Projects the gravity vector into the base frame
        */
        Eigen::Vector3d gravity_w(0.0, 0.0, -9.81);
        gravity_proj = base_w.inverse() * gravity_w;
    }




}; // namespace rl_utils