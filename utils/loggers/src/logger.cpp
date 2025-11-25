#include "loggers/logger.hpp"

namespace loggers
{
Logger::Logger(const std::string & node_name, 
    double publish_rate_hz)
{
    node_ptr_ = rclcpp::Node::make_shared(node_name);
    debug_publisher_ptr_ = node_ptr_->create_publisher<common_msgs::msg::Debug>(
        "/log", rclcpp::SystemDefaultsQoS());
    // Timer
    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ptr_ = node_ptr_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
        std::bind(&Logger::publish_callback, this));
    // Executor
    executor_.add_node(node_ptr_);

}

Logger::~Logger() {
    stop();
}

void Logger::register_values(ValueConfig value_config) {
    // Register headers
    s_.msg.source_name = value_config.source_name;
    s_.msg.labels = value_config.labels;
    s_.msg.values.resize(value_config.value_ptrs.size(), 0.0);
    // Register points
    s_.value_ptrs = value_config.value_ptrs;
}

void Logger::start() {
    if(is_running_) {
        return;
    }
    is_running_ = true;
    // Check if thread exists and has finished
    if(exec_thread_.joinable()) {
        exec_thread_.join();  // Wait for previous thread to finish
    }
    
    // Create new thread (works for both first time and restart)
    exec_thread_ = std::thread([this]() {
        executor_.spin();
    });
} 

void Logger::pause() {
    if(!is_running_) {
        return;
    }
    is_running_ = false;
    executor_.cancel();
}

void Logger::stop() {
    if(!is_running_) {
        return;
    }
    is_running_ = false;
    executor_.cancel();
    executor_.remove_node(node_ptr_);

    if(exec_thread_.joinable()) {
        exec_thread_.join();
    }

}

void Logger::publish_callback() {
    // Update values
    for (size_t i = 0; i < s_.value_ptrs.size(); ++i) {
        if(s_.value_ptrs[i] == nullptr){
            s_.msg.values[i] = std::numeric_limits<double>::quiet_NaN(); 
            continue;
        }
        s_.msg.values[i] = *(s_.value_ptrs[i]);
    }
    // Update timestamp
    s_.msg.stamp = node_ptr_->now();
    // Publish
    debug_publisher_ptr_->publish(s_.msg);
}

}; // namespace loggers