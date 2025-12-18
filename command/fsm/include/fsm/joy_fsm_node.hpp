#pragma once
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "fsm/fsm.hpp"



namespace fsm
{
    class FSMNode : public rclcpp::Node
    {
    public:
        FSMNode() : rclcpp::Node("joy_fsm_node") {
            // initialize 
            get_params();
            // subscriber
            auto qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
            joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                joy_topic_, qos,
                std::bind(&FSMNode::joy_callback, this, std::placeholders::_1)
            );
        }

        void initialize() {
            // Initialize FSM after the shared_ptr is created
            std::vector<std::list<std::string>> controller_chains;
            for (const auto& pair : key_controller_map_) {
                if (!pair.second.empty())
                {
                    controller_chains.push_back(std::list<std::string>(pair.second.begin(), pair.second.end()));
                }
            }
            fsm_ptr_ = std::make_shared<FSM>(controller_chains);
            fsm_ptr_->get_controller_states();
        }

        void get_params(){ 
            std::vector<int> keys = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};  
    
            for (int key : keys) {
                std::string param_name = "key_controller_map." + std::to_string(key);
                this->declare_parameter(param_name, std::vector<std::string>{});
                
                if (this->has_parameter(param_name)) {
                    auto controllers = this->get_parameter(param_name).as_string_array();
                    // Only add to map if it has controllers
                    if (!controllers.empty()) {
                        key_controller_map_[key] = controllers;
                    }
                }
            }
            print_key_controller_map();
        }

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
            // process button states
            for (size_t i = 0; i < button_states_.size() && i < msg->buttons.size(); ++i) {
                bool current_state = msg->buttons[i] != 0;
                if (current_state && !button_states_[i]) {
                    if (key_controller_map_.find(i) != key_controller_map_.end()) {
                        auto request = ControllerSwitchRequest(); 
                        request.start_controllers.clear();
                        for (const auto& controller_name : key_controller_map_[i]) {
                            request.start_controllers.push_back(controller_name);
                        }
                        fsm_ptr_->switch_controllers(request);
                    }
                    else {
                        RCLCPP_WARN(this->get_logger(), "No controllers mapped to key %d", i);
                    }
                }
                button_states_[i] = current_state;
            }
        }

        void update_controller_states() {
            if (fsm_ptr_) {
                fsm_ptr_->get_controller_states();
            }
        }

        void print_key_controller_map() {
            for (const auto& [key, controllers] : key_controller_map_) {
                std::string controllers_str;
                for (const auto& controller : controllers) {
                    controllers_str += controller + ", ";
                }
                if (!controllers_str.empty()) {
                    controllers_str = controllers_str.substr(0, controllers_str.length() - 2); 
                }
                RCLCPP_INFO(this->get_logger(), "  Key %d: [%s]", key, controllers_str.c_str());
            }
        }

        std::shared_ptr<FSM> get_fsm() {
            return fsm_ptr_;
        }

    protected:
        rclcpp::TimerBase::SharedPtr update_timer_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_ = nullptr;
        std::string joy_topic_ = "/joy";
        std::array<bool, 10> button_states_ = {false, false, false,
                                               false, false, false, 
                                               false, false, false, false};
        std::map<int, std::vector<std::string>> key_controller_map_;
        std::shared_ptr<FSM> fsm_ptr_ = nullptr;
    }; 

};