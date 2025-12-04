#include "mujoco_sim.h"
#include "rclcpp/rclcpp.hpp"
#include <signal.h> 
#include <stdexcept>

#include <pluginlib/class_loader.hpp>
#include "mujoco_sim_node_base.hpp"

using namespace std::chrono_literals;
using namespace mujoco_sim;

void handle_ctrl_c(int signal) {
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <robot_type> <mjcf_path> <ground_truth>" << std::endl;
        return 1;
    }
    std::string mjcf_path = std::string(argv[1]);
    std::string node_name = std::string(argv[2]);
    node_name = "mujoco_sim::"+node_name;
    std::cout << "load xml from path: " << mjcf_path << std::endl;

    MujocoSim mujoco_sim_(mjcf_path);
    mj::Simulate* sim_ptr = mujoco_sim_.getSimPtr();

    rclcpp::init(argc, argv);
    signal(SIGINT, handle_ctrl_c);

    // load ROS2 nodes as plugins 
    pluginlib::ClassLoader<MujocoSimNodeBase> loader("mujoco_sim", "mujoco_sim::MujocoSimNodeBase");
    std::shared_ptr<MujocoSimNodeBase> mujoco_ros2_node_ptr;
    try {
        mujoco_ros2_node_ptr = loader.createSharedInstance(node_name);
    } catch (const pluginlib::LibraryLoadException& ex) {
        std::cerr << "Failed to load node plugin: " << ex.what() << std::endl;
        return 1;
    }
    mujoco_ros2_node_ptr->set_sim_ptr(sim_ptr);
    std::function<void(std::shared_ptr<MujocoSimNodeBase>)> spin_func;
    spin_func = [](std::shared_ptr<MujocoSimNodeBase> node_ptr){
        rclcpp::spin(node_ptr); // spin constantly, the message frequencey is set based on timmer;
    };

    std::thread physicsthreadhandle(&MujocoSim::PhysicsThread, &mujoco_sim_);
    auto spin_thread = std::thread{spin_func, mujoco_ros2_node_ptr};
    sim_ptr->RenderLoop();
    // thread ending sequence: render ->spin -> physics
    spin_thread.join();
    physicsthreadhandle.join();

    return 0;
}