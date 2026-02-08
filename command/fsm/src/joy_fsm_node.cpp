#include "fsm/joy_fsm_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<fsm::FSMNode>();
  node->initialize();

  executor.add_node(node);
  executor.add_node(node->get_fsm()->get_node());

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
