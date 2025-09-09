#include "mujoco_sim.h"
#include "rclcpp/rclcpp.hpp"
#include <signal.h> 
#include <stdexcept>

#include "robot_specific/go2/go2_sim_node.hpp"
// #include "robot_specific/pogox/pogox_sim_node.hpp"

using namespace std::chrono_literals;
using namespace mujoco_sim;

void handle_ctrl_c(int signal) {
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <robot_type> <mjcf_path> <ground_truth>" << std::endl;
        return 1;
    }
    std::string mjcf_path = std::string(argv[1]);
    std::string robot_type = std::string(argv[2]);
    bool ground_truth = (std::string(argv[3]) == "true");
    std::cout << "load xml from path: " << mjcf_path << " for robot: " << robot_type << std::endl;

    MujocoSim mujoco_sim_(mjcf_path);
    mj::Simulate* sim_ptr = mujoco_sim_.getSimPtr();

    rclcpp::init(argc, argv);
    signal(SIGINT, handle_ctrl_c);

    std::shared_ptr<rclcpp::Node> mujoco_ros2_node_ptr;
    std::function<void(std::shared_ptr<rclcpp::Node>)> spin_func;
    spin_func = [](std::shared_ptr<rclcpp::Node> node_ptr){
        rclcpp::spin(node_ptr); // spin constantly, the message frequencey is set based on timmer;
    };

    if(robot_type == "go2"){
        if(!ground_truth){
            mujoco_ros2_node_ptr = std::make_shared<Go2SimNode>(sim_ptr);

        }
        else if(ground_truth){
            // auto mujoco_ros2_node_ptr = std::make_shared<Go2SimGroundTruth>(sim_ptr);
            throw std::logic_error("Ground truth simulation not implemented for " + robot_type);
        }
        else{
            std::cerr << "Unsupported flag!" << std::endl;
            return 1;
        }
    }
    else{
        std::cerr << "Unsupported robot type!" << std::endl;
        return 1;
    }

    std::thread physicsthreadhandle(&MujocoSim::PhysicsThread, &mujoco_sim_);
    auto spin_thread = std::thread{spin_func, mujoco_ros2_node_ptr};
    sim_ptr->RenderLoop();
    // thread ending sequence: render ->spin -> physics
    spin_thread.join();
    physicsthreadhandle.join();

    return 0;
}