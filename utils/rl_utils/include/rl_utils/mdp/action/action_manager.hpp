#pragma once
#ifndef ACTION_MANAGER_HPP_
#define ACTION_MANAGER_HPP_

#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace action
{

struct ActionTermCfg
{
  std::string name{};
  int action_dim{0};
  std::vector<double> action_scale{};
  std::vector<double> action_offset{};
};

class ActionTerm
{
public:
  explicit ActionTerm(ActionTermCfg cfg) : cfg_(std::move(cfg)) {}
  virtual ~ActionTerm() = default;

  virtual int action_dim() const = 0;
  virtual std::vector<double> raw_actions() const = 0;
  virtual std::vector<double> processed_actions() const = 0;
  virtual void process_actions(const std::vector<double>& actions) = 0;
  virtual void reset() {}

protected:
  ActionTermCfg cfg_;
};

class ActionManager
{
public:
  ActionManager() = default;

  void add_term(std::unique_ptr<ActionTerm> term)
  {
    terms_.push_back(std::move(term));
    action_.assign(total_action_dim(), 0.0f);
  }

  void reset()
  {
    action_.assign(total_action_dim(), 0.0f);
    for (auto& term : terms_)
    {
      term->reset();
    }
  }

  std::vector<double> action() const
  {
    return action_;
  }

  std::vector<double> raw_actions() const
  {
    std::vector<double> actions;
    actions.reserve(total_action_dim());
    for (const auto& term : terms_)
    {
      const auto term_actions = term->raw_actions();
      actions.insert(actions.end(), term_actions.begin(), term_actions.end());
    }
    return actions;
  }

  std::vector<double> processed_actions() const
  {
    std::vector<double> actions;
    actions.reserve(total_action_dim());
    for (const auto& term : terms_)
    {
      const auto term_actions = term->processed_actions();
      actions.insert(actions.end(), term_actions.begin(), term_actions.end());
    }
    return actions;
  }

  void process_action(const std::vector<double>& action)
  {
    action_ = action;

    int idx = 0;
    for (auto& term : terms_)
    {
      const int dim = term->action_dim();
      const auto term_action = std::vector<double>(action.begin() + idx, action.begin() + idx + dim);
      term->process_actions(term_action);
      idx += dim;
    }
  }

  int total_action_dim() const
  {
    const auto dims = action_dim();
    return std::accumulate(dims.begin(), dims.end(), 0);
  }

  std::vector<int> action_dim() const
  {
    std::vector<int> dims;
    dims.reserve(terms_.size());
    for (const auto& term : terms_)
    {
      dims.push_back(term->action_dim());
    }
    return dims;
  }

private:
  std::vector<double> action_;
  std::vector<std::unique_ptr<ActionTerm>> terms_;
};

}  // namespace action

#endif  // ACTION_MANAGER_HPP_
