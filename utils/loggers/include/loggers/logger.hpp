#pragma once

#include <string>
#include <vector>

#include "common_msgs/msg/debug.hpp"
#include "rclcpp/rclcpp.hpp"

namespace loggers
{
struct ValueConfig
{
  std::string source_name;
  std::vector<std::string> labels;
  std::vector<double*> value_ptrs;
};

struct Signal
{
  common_msgs::msg::Debug msg;
  std::vector<double*> value_ptrs;
};

class Logger
{
  public:
  Logger(const std::string& node_name, double publish_rate_hz);

  ~Logger();

  void register_values(ValueConfig value_config);

  void start();

  void pause();

  void stop();

  protected:
  void publish_callback();

  Signal s_;

  rclcpp::Node::SharedPtr node_ptr_ = nullptr;
  rclcpp::Publisher<common_msgs::msg::Debug>::SharedPtr debug_publisher_ptr_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_ptr_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
  bool is_running_ = false;
  std::thread exec_thread_;
};

};  // namespace loggers
