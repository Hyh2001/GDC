#include "keyboard/keyboard.hpp"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardNode>());
  rclcpp::shutdown();
  return 0;
}