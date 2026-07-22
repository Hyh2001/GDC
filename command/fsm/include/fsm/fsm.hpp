#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fsm
{
struct ControllerNode
{
  std::string name;
  bool active{false};
  std::shared_ptr<ControllerNode> next{nullptr};
};

struct ControllerSwitchRequest
{
  std::vector<std::string> start_controllers;
  int strictness{controller_manager_msgs::srv::SwitchController::Request::STRICT};
};

enum class ControllerOperationState
{
  IDLE,
  REFRESHING,
  QUERYING_BEFORE_SWITCH,
  SWITCHING
};

class FSM
{
  public:
  explicit FSM(const std::vector<std::list<std::string>>& controller_chains);

  void get_controller_states();
  void switch_controllers(const ControllerSwitchRequest& request);
  std::vector<std::string> get_active_controllers() const;

  rclcpp::Node::SharedPtr get_node()
  {
    return node_ptr_;
  }

  private:
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  std::vector<std::string> get_active_controllers_locked() const;
  void update_controller_states_locked(const ListControllers::Response& response);
  void reset_operation_locked();

  void query_states_before_switch();
  void send_switch_request(const std::vector<std::string>& controllers_to_activate,
                           const std::vector<std::string>& controllers_to_deactivate,
                           const std::vector<std::string>& desired_controllers, int strictness);

  std::vector<std::shared_ptr<ControllerNode>> controllers_{};
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::TimerBase::SharedPtr state_update_timer_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_ptr_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_ptr_;

  // All members below, plus ControllerNode::active, are protected by this mutex.
  mutable std::mutex state_mutex_;
  ControllerOperationState operation_state_{ControllerOperationState::IDLE};
  std::vector<std::string> pending_start_controllers_{};
  int pending_strictness_{SwitchController::Request::STRICT};
};
}  // namespace fsm
