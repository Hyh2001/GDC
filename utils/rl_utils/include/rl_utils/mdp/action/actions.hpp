#pragma once
#ifndef ACTIONS_HPP_
#define ACTIONS_HPP_

#include <stdexcept>
#include "rl_utils/mdp/action/action_manager.hpp"

namespace action
{

  class JointPositionActionTerm : public action::ActionTerm
  {
  public:
    explicit JointPositionActionTerm(const action::ActionTermCfg& cfg) : ActionTerm(cfg) {
      if (cfg_.action_scale.size() != cfg_.action_dim || cfg_.action_offset.size() != cfg_.action_dim)
      {
        throw std::invalid_argument("action_scale and action_offset must have the same size to act_dim.");
      }
      raw_actions_.assign(cfg_.action_dim, 0.0);
      processed_actions_.assign(cfg_.action_dim, 0.0);
    }

    int action_dim() const override { return cfg_.action_dim; }

    std::vector<double> raw_actions() const override { return raw_actions_; }

    std::vector<double> processed_actions() const override { return processed_actions_; }

    void process_actions(const std::vector<double>& actions) override
    {
      raw_actions_ = actions;
      for (size_t i = 0; i < actions.size(); ++i)
      {
        processed_actions_[i] = actions[i] * cfg_.action_scale[i] + cfg_.action_offset[i];
      }
    }

  private:
    std::vector<double> raw_actions_;
    std::vector<double> processed_actions_;
  };

  // TODO: add joint velocity action term, joint torque action term, joint impedance action term

} // namespace action

#endif  // ACTIONS_HPP_
