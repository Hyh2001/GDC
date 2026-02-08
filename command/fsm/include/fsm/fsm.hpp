#pragma once
#include <vector>
#include <list>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"

namespace fsm
{
    struct ControllerNode
    {
        std::string name;
        bool active;
        std::shared_ptr<ControllerNode> next=nullptr;
    };

    struct ControllerSwitchRequest
    {
        std::vector<std::string> start_controllers;
        int strictness=1; // 1: BEST_EFFORT, 2: STRICT
    };

    class FSM
    {
    public:
        FSM(const std::vector<std::list<std::string>> & controller_chains);

        void get_controller_states();
        void switch_controllers(const ControllerSwitchRequest & request);
        bool is_switchable(const std::vector<std::string> start_controllers,
            std::vector<std::string> stop_controllers);
        std::vector<std::string> get_active_controllers();

        rclcpp::Node::SharedPtr get_node() { return node_ptr_; }

    protected:
        std::vector<std::shared_ptr<ControllerNode>> controllers_{};

        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::TimerBase::SharedPtr state_update_timer_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_ptr_;
        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_ptr_;
    };
}
