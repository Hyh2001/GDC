#include "fsm/fsm.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <unordered_map>

namespace fsm
{
namespace
{
using namespace std::chrono_literals;

constexpr auto kStateRefreshPeriod = 1s;
constexpr auto kServiceWaitTimeout = 2s;
constexpr int32_t kSwitchTimeoutSeconds = 3;

bool contains(const std::vector<std::string>& names, const std::string& name)
{
  return std::find(names.begin(), names.end(), name) != names.end();
}

std::string format_controller_names(const std::vector<std::string>& names)
{
  std::ostringstream stream;
  for (size_t i = 0; i < names.size(); ++i)
  {
    if (i > 0)
    {
      stream << ", ";
    }
    stream << names[i];
  }
  return stream.str();
}
}  // namespace

FSM::FSM(const std::vector<std::list<std::string>>& controller_chains)
{
  node_ptr_ = rclcpp::Node::make_shared("fsm_internal_node");
  switch_client_ptr_ = node_ptr_->create_client<SwitchController>("/controller_manager/switch_controller");
  list_client_ptr_ = node_ptr_->create_client<ListControllers>("/controller_manager/list_controllers");

  std::unordered_map<std::string, std::shared_ptr<ControllerNode>> controllers_by_name;
  for (const auto& chain : controller_chains)
  {
    std::shared_ptr<ControllerNode> previous_controller;
    for (const auto& name : chain)
    {
      auto& controller = controllers_by_name[name];
      if (!controller)
      {
        controller = std::make_shared<ControllerNode>();
        controller->name = name;
        controllers_.push_back(controller);
      }

      if (previous_controller)
      {
        previous_controller->next = controller;
      }
      previous_controller = controller;
    }
  }

  state_update_timer_ = node_ptr_->create_wall_timer(kStateRefreshPeriod, std::bind(&FSM::get_controller_states, this));
}

std::vector<std::string> FSM::get_active_controllers_locked() const
{
  std::vector<std::string> active_controllers;
  for (const auto& controller : controllers_)
  {
    if (controller->active)
    {
      active_controllers.push_back(controller->name);
    }
  }
  return active_controllers;
}

std::vector<std::string> FSM::get_active_controllers() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return get_active_controllers_locked();
}

void FSM::update_controller_states_locked(const ListControllers::Response& response)
{
  for (auto& controller : controllers_)
  {
    const auto state =
        std::find_if(response.controller.begin(), response.controller.end(),
                     [&controller](const auto& candidate) { return candidate.name == controller->name; });
    controller->active = state != response.controller.end() && state->state == "active";
  }
}

void FSM::reset_operation_locked()
{
  pending_start_controllers_.clear();
  operation_state_ = ControllerOperationState::IDLE;
}

void FSM::get_controller_states()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (operation_state_ != ControllerOperationState::IDLE)
    {
      return;
    }
    operation_state_ = ControllerOperationState::REFRESHING;
  }

  if (!list_client_ptr_->wait_for_service(kServiceWaitTimeout))
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    reset_operation_locked();
    RCLCPP_WARN(node_ptr_->get_logger(), "ListControllers service is unavailable.");
    return;
  }

  list_client_ptr_->async_send_request(std::make_shared<ListControllers::Request>(),
                                       [this](rclcpp::Client<ListControllers>::SharedFuture future)
                                       {
                                         try
                                         {
                                           const auto response = future.get();
                                           std::lock_guard<std::mutex> lock(state_mutex_);
                                           if (operation_state_ != ControllerOperationState::REFRESHING)
                                           {
                                             return;
                                           }
                                           update_controller_states_locked(*response);
                                           reset_operation_locked();
                                         }
                                         catch (const std::exception& error)
                                         {
                                           std::lock_guard<std::mutex> lock(state_mutex_);
                                           reset_operation_locked();
                                           RCLCPP_ERROR(node_ptr_->get_logger(),
                                                        "Failed to refresh controller states: %s", error.what());
                                         }
                                       });
}

void FSM::switch_controllers(const ControllerSwitchRequest& request)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (operation_state_ != ControllerOperationState::IDLE)
    {
      RCLCPP_WARN(node_ptr_->get_logger(),
                  "A controller-manager operation is already in progress; ignoring switch request.");
      return;
    }

    pending_start_controllers_ = request.start_controllers;
    pending_strictness_ = request.strictness;
    operation_state_ = ControllerOperationState::QUERYING_BEFORE_SWITCH;
  }

  query_states_before_switch();
}

void FSM::query_states_before_switch()
{
  if (!list_client_ptr_->wait_for_service(kServiceWaitTimeout))
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    reset_operation_locked();
    RCLCPP_WARN(node_ptr_->get_logger(), "ListControllers service is unavailable.");
    return;
  }

  list_client_ptr_->async_send_request(
      std::make_shared<ListControllers::Request>(),
      [this](rclcpp::Client<ListControllers>::SharedFuture future)
      {
        std::vector<std::string> controllers_to_activate;
        std::vector<std::string> controllers_to_deactivate;
        std::vector<std::string> desired_controllers;
        int strictness = SwitchController::Request::STRICT;

        try
        {
          const auto response = future.get();
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (operation_state_ != ControllerOperationState::QUERYING_BEFORE_SWITCH)
            {
              return;
            }

            update_controller_states_locked(*response);
            desired_controllers = pending_start_controllers_;
            strictness = pending_strictness_;

            const auto active_controllers = get_active_controllers_locked();
            for (const auto& controller : desired_controllers)
            {
              if (!contains(active_controllers, controller))
              {
                controllers_to_activate.push_back(controller);
              }
            }
            for (const auto& controller : active_controllers)
            {
              if (!contains(desired_controllers, controller))
              {
                controllers_to_deactivate.push_back(controller);
              }
            }

            if (controllers_to_activate.empty() && controllers_to_deactivate.empty())
            {
              RCLCPP_INFO(node_ptr_->get_logger(), "Requested controllers are already active.");
              reset_operation_locked();
              return;
            }
            operation_state_ = ControllerOperationState::SWITCHING;
          }

          send_switch_request(controllers_to_activate, controllers_to_deactivate, desired_controllers, strictness);
        }
        catch (const std::exception& error)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          reset_operation_locked();
          RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to query controller states before switching: %s", error.what());
        }
      });
}

void FSM::send_switch_request(const std::vector<std::string>& controllers_to_activate,
                              const std::vector<std::string>& controllers_to_deactivate,
                              const std::vector<std::string>& desired_controllers, int strictness)
{
  if (!switch_client_ptr_->wait_for_service(kServiceWaitTimeout))
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    reset_operation_locked();
    RCLCPP_WARN(node_ptr_->get_logger(), "SwitchController service is unavailable.");
    return;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Switching controllers - Start: [%s] Stop: [%s]",
              format_controller_names(controllers_to_activate).c_str(),
              format_controller_names(controllers_to_deactivate).c_str());

  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = controllers_to_activate;
  request->deactivate_controllers = controllers_to_deactivate;
  request->strictness = strictness;
  request->timeout.sec = kSwitchTimeoutSeconds;
  request->timeout.nanosec = 0;

  switch_client_ptr_->async_send_request(
      request,
      [this, desired_controllers](rclcpp::Client<SwitchController>::SharedFuture future)
      {
        bool success = false;
        try
        {
          success = future.get()->ok;
        }
        catch (const std::exception& error)
        {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Controller switch raised an exception: %s", error.what());
        }

        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (success)
          {
            for (auto& controller : controllers_)
            {
              controller->active = contains(desired_controllers, controller->name);
            }
          }
          reset_operation_locked();
        }

        if (success)
        {
          RCLCPP_INFO(node_ptr_->get_logger(), "Controller switch successful.");
        }
        else
        {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Controller switch failed.");
        }

        // Reconcile the cache with the controller manager after every attempt.
        get_controller_states();
      });
}
}  // namespace fsm
