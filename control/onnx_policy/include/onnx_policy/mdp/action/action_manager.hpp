#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace onnx_policy::action
{
struct ActionTermCfg
{
  std::string name{};
  int action_dim{0};
  std::vector<double> action_scale{}, action_offset{};
};

class ActionTerm
{
  public:
  explicit ActionTerm(ActionTermCfg cfg) : cfg_(std::move(cfg))
  {
    if (cfg_.action_dim <= 0)
      throw std::invalid_argument("Positive action dimension required");
    if (cfg_.action_scale.size() != static_cast<size_t>(cfg_.action_dim) ||
        cfg_.action_offset.size() != static_cast<size_t>(cfg_.action_dim))
      throw std::invalid_argument("Action scale/offset size mismatch");
    for (double value : cfg_.action_scale)
      if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite action scale");
    for (double value : cfg_.action_offset)
      if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite action offset");
  }
  virtual ~ActionTerm() = default;
  virtual int action_dim() const = 0;
  virtual std::vector<double> raw_actions() const = 0;
  virtual std::vector<double> processed_actions() const = 0;
  virtual void process_actions(const std::vector<double>& actions) = 0;
  virtual void reset()
  {
  }
  // Existing custom terms need not implement a new accessor.
  virtual void copy_processed_actions(std::vector<double>& output) const
  {
    const auto values = processed_actions();
    if (values.size() != output.size())
      throw std::invalid_argument("Action term output size mismatch");
    std::copy(values.begin(), values.end(), output.begin());
  }

  protected:
  ActionTermCfg cfg_;
};

class ActionManager
{
  public:
  void add_term(std::unique_ptr<ActionTerm> term)
  {
    if (!term || term->action_dim() <= 0)
      throw std::invalid_argument("Invalid action term");
    if (term->action_dim() > std::numeric_limits<int>::max() - total_action_dim())
      throw std::invalid_argument("Action dimension overflow");
    term_inputs_.emplace_back(term->action_dim(), 0.0);
    term_outputs_.emplace_back(term->action_dim(), 0.0);
    terms_.push_back(std::move(term));
    action_.resize(total_action_dim());
    processed_.resize(total_action_dim());
  }
  void reset()
  {
    std::fill(action_.begin(), action_.end(), 0.0);
    std::fill(processed_.begin(), processed_.end(), 0.0);
    for (auto& term : terms_) term->reset();
    for (auto& values : term_inputs_) std::fill(values.begin(), values.end(), 0.0);
    for (auto& values : term_outputs_) std::fill(values.begin(), values.end(), 0.0);
  }
  std::vector<double> action() const
  {
    return action_;
  }
  std::vector<double> raw_actions() const
  {
    return action_;
  }
  std::vector<double> processed_actions() const
  {
    return processed_;
  }
  const std::vector<double>& processed_actions_view() const
  {
    return processed_;
  }

  void process_action(const std::vector<double>& values)
  {
    if (values.size() != action_.size())
      throw std::invalid_argument("Action manager input size mismatch");
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite action input");
    size_t offset = 0;
    for (size_t i = 0; i < terms_.size(); ++i)
    {
      auto& input = term_inputs_[i];
      std::copy_n(values.begin() + offset, input.size(), input.begin());
      terms_[i]->process_actions(input);
      terms_[i]->copy_processed_actions(term_outputs_[i]);
      for (double value : term_outputs_[i])
        if (!std::isfinite(value))
          throw std::invalid_argument("Nonfinite processed action");
      offset += input.size();
    }
    std::copy(values.begin(), values.end(), action_.begin());
    offset = 0;
    for (const auto& output : term_outputs_)
    {
      std::copy(output.begin(), output.end(), processed_.begin() + offset);
      offset += output.size();
    }
  }
  int total_action_dim() const
  {
    int total = 0;
    for (const auto& term : terms_) total += term->action_dim();
    return total;
  }
  std::vector<int> action_dim() const
  {
    std::vector<int> dimensions;
    for (const auto& term : terms_) dimensions.push_back(term->action_dim());
    return dimensions;
  }

  private:
  std::vector<std::unique_ptr<ActionTerm>> terms_;
  std::vector<std::vector<double>> term_inputs_, term_outputs_;
  std::vector<double> action_, processed_;
};
}  // namespace onnx_policy::action
