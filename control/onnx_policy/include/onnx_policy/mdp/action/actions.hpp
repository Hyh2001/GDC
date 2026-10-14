#pragma once

#include "onnx_policy/mdp/action/action_manager.hpp"

namespace onnx_policy::action
{
class JointPositionActionTerm : public ActionTerm
{
  public:
  explicit JointPositionActionTerm(const ActionTermCfg& cfg) : ActionTerm(cfg)
  {
    raw_actions_.resize(cfg_.action_dim);
    processed_actions_.resize(cfg_.action_dim);
  }
  int action_dim() const override
  {
    return cfg_.action_dim;
  }
  std::vector<double> raw_actions() const override
  {
    return raw_actions_;
  }
  std::vector<double> processed_actions() const override
  {
    return processed_actions_;
  }
  void process_actions(const std::vector<double>& values) override
  {
    if (values.size() != raw_actions_.size())
      throw std::invalid_argument("Action term input size mismatch");
    // Validate the whole term before changing its stored outputs.
    for (size_t i = 0; i < values.size(); ++i)
      if (!std::isfinite(values[i]) || !std::isfinite(values[i] * cfg_.action_scale[i] + cfg_.action_offset[i]))
        throw std::invalid_argument("Nonfinite action value");
    std::copy(values.begin(), values.end(), raw_actions_.begin());
    for (size_t i = 0; i < values.size(); ++i)
      processed_actions_[i] = values[i] * cfg_.action_scale[i] + cfg_.action_offset[i];
  }
  void reset() override
  {
    std::fill(raw_actions_.begin(), raw_actions_.end(), 0.0);
    std::fill(processed_actions_.begin(), processed_actions_.end(), 0.0);
  }
  void copy_processed_actions(std::vector<double>& output) const override
  {
    if (output.size() != processed_actions_.size())
      throw std::invalid_argument("Action output size mismatch");
    std::copy(processed_actions_.begin(), processed_actions_.end(), output.begin());
  }

  protected:
  std::vector<double> raw_actions_, processed_actions_;
};

class JointVelocityActionTerm : public JointPositionActionTerm
{
  public:
  explicit JointVelocityActionTerm(const ActionTermCfg& cfg) : JointPositionActionTerm(cfg)
  {
  }
};
class JointEffortActionTerm : public JointPositionActionTerm
{
  public:
  explicit JointEffortActionTerm(const ActionTermCfg& cfg) : JointPositionActionTerm(cfg)
  {
    std::fill(cfg_.action_offset.begin(), cfg_.action_offset.end(), 0.0);
  }
};
class JointImpedanceActionTerm : public JointPositionActionTerm
{
  public:
  explicit JointImpedanceActionTerm(const ActionTermCfg& cfg) : JointPositionActionTerm(cfg)
  {
  }
};
}  // namespace onnx_policy::action
