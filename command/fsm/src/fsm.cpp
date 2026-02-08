#include "fsm/fsm.hpp"

namespace fsm
{
    FSM::FSM(const std::vector<std::list<std::string>> & controller_chains)
    {
        // Create internal node for service calls
        node_ptr_ = rclcpp::Node::make_shared("fsm_internal_node");

        switch_client_ptr_ = node_ptr_->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
        list_client_ptr_ = node_ptr_->create_client<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers");

        std::unordered_map<std::string, std::shared_ptr<ControllerNode>> name_to_node;

        // Create unique controller nodes
        for (const auto& chain : controller_chains) {
            std::shared_ptr<ControllerNode> prev = nullptr;
            for (const auto& name : chain) {
                std::shared_ptr<ControllerNode> node;
                auto it = name_to_node.find(name);
                if (it == name_to_node.end()) {
                    node = std::make_shared<ControllerNode>();
                    node->name = name;
                    node->active = false;
                    controllers_.push_back(node);
                    name_to_node[name] = node;
                } else {
                    node = it->second;
                }
                if (prev) {
                    prev->next = node;
                }
                prev = node;
            }
        }

        state_update_timer_ = node_ptr_->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&FSM::get_controller_states, this)  // Use async version
        );
    }

    void FSM::get_controller_states()
    {
        auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

        if (!list_client_ptr_->wait_for_service(std::chrono::seconds(2)))
        {
            std::cout << "[FSM] Service /controller_manager/list_controllers not available." << std::endl;
            return;
        }

        auto callback = [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future)
        {
            try {
                auto response = future.get();

                for (auto & controller : controllers_)
                {
                    controller->active = false;
                    for (const auto & state : response->controller)
                    {
                        if (state.name == controller->name)
                        {
                            controller->active = (state.state == "active");
                            break;
                        }
                    }
                }
            } catch (const std::exception& e) {
                std::cout << "[FSM] Failed to get controller states: " << e.what() << std::endl;
            }
        };

        list_client_ptr_->async_send_request(request, callback);
    }

    void FSM::switch_controllers(const ControllerSwitchRequest & request)
    {
        bool switchable = is_switchable(request.start_controllers, get_active_controllers());
        if (!switchable)
        {
            std::cout << "[FSM] WARNING: Requested controllers cannot be switched ." << std::endl;
            return;
        }

        if (!switch_client_ptr_->wait_for_service(std::chrono::seconds(2)))
        {
            std::cout << "[FSM] Service /controller_manager/switch_controller not available." << std::endl;
            return ;
        }

        const auto active_controllers = get_active_controllers();
        const auto start_controllers = request.start_controllers;

        std::cout << "[FSM] Switching controllers (two-stage) - Start: [";
        for (const auto& c : start_controllers) std::cout << c << " ";
        std::cout << "] Stop: [";
        for (const auto& c : active_controllers) std::cout << c << " ";
        std::cout << "]" << std::endl;

        auto send_activate = [this, start_controllers, request]() {
            if (start_controllers.empty()) {
                this->get_controller_states();
                return;
            }
            auto activate_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            activate_req->activate_controllers = start_controllers;
            activate_req->strictness = request.strictness;
            switch_client_ptr_->async_send_request(
                activate_req,
                [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->ok) {
                            std::cout << "[FSM] ✓ Controller activation successful." << std::endl;
                            this->get_controller_states();
                        } else {
                            std::cout << "[FSM] ✗ Controller activation failed." << std::endl;
                        }
                    } catch (const std::exception& e) {
                        std::cout << "[FSM] Exception during controller activation: " << e.what() << std::endl;
                    }
                });
        };

        if (active_controllers.empty()) {
            send_activate();
            return;
        }

        auto deactivate_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        deactivate_req->deactivate_controllers = active_controllers;
        deactivate_req->strictness = request.strictness;

        switch_client_ptr_->async_send_request(
            deactivate_req,
            [this, send_activate](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->ok) {
                        std::cout << "[FSM] ✓ Controller deactivation successful." << std::endl;
                        send_activate();
                    } else {
                        std::cout << "[FSM] ✗ Controller deactivation failed." << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cout << "[FSM] Exception during controller deactivation: " << e.what() << std::endl;
                }
            });
    }

    bool FSM::is_switchable(const std::vector<std::string> start_controllers,
        std::vector<std::string> /*stop_controllers*/)
    {
        for (const auto& chain_head : controllers_)
        {
            std::shared_ptr<ControllerNode> current = chain_head;
            bool found_started_in_chain = false;
            while (current)
            {
                if (std::find(start_controllers.begin(), start_controllers.end(), current->name) != start_controllers.end())
                {
                    if (found_started_in_chain)
                    {
                        return false;
                    }
                    found_started_in_chain = true;
                }
                current = current->next;
            }
        }
        auto active_controllers = get_active_controllers();
        for (const auto& controller_to_start : start_controllers)
        {
            if (std::find(active_controllers.begin(), active_controllers.end(), controller_to_start) != active_controllers.end())
            {
                std::cout << "[FSM] WARNING: Controller '" << controller_to_start << "' is already active. Skipping switch." << std::endl;
                return false;
            }
        }
        return true;
    }

    std::vector<std::string> FSM::get_active_controllers()
    {
        std::vector<std::string> active_controllers;
        for (const auto & controller : controllers_)
        {
            if (controller->active)
            {
                active_controllers.push_back(controller->name);
            }
        }
        return active_controllers;
    }
}
