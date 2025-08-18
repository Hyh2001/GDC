// #include "../../mujoco_sim.h"
// #include "rclcpp/rclcpp.hpp"
// #include "robot_specific/pogox/pogox_sim_node.h"
// #include <signal.h> 

// using namespace std::chrono_literals;
// using namespace mujoco_sim;

// void handle_ctrl_c(int signal) {
//     rclcpp::shutdown();
//     exit(0);
// }

// int main(int argc, char** argv) {

//     std::string mjcf_path = "";
//     if (argc >  1) {
//         mjcf_path = std::string(argv[1]);
//     }
//     std::cout << "load xml from path: " << mjcf_path << std::endl;

//     MujocoSim mujoco_sim_(mjcf_path);
//     mj::Simulate* sim_ptr = mujoco_sim_.getSimPtr();

//     rclcpp::init(argc, argv);
//     signal(SIGINT, handle_ctrl_c);
//     auto mujoco_ros2_node_ptr = std::make_shared<PogoXSimGroundTruth>(sim_ptr);
//     auto spin_func = [](std::shared_ptr<PogoXSimGroundTruth> node_ptr){
//        rclcpp::spin(node_ptr); // spin constantly, the message frequencey is set based on timmer;
//     };
//     // std::cout << "pogox_sim is successfully setup" << std::endl;
//     std::thread physicsthreadhandle(&MujocoSim::PhysicsThread, &mujoco_sim_);
//     auto spin_thread = std::thread{spin_func, mujoco_ros2_node_ptr};
//     // std::cout << "pogox_sim is successfully spin" << std::endl;
//     sim_ptr->RenderLoop();
//     // std::cout << "pogox_sim is successfully rendering" << std::endl;
//     // thread ending sequence: render ->spin -> physics
//     spin_thread.join();
//     physicsthreadhandle.join();

//     return 0;
// }