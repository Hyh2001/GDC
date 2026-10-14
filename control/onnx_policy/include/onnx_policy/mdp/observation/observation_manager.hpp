#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace onnx_policy::observation
{
struct ObservationTermCfg
{
  std::string name{};
  int observation_dim{0};
  std::vector<double> scale{}, clip_min{}, clip_max{};
  int history_length{1};
};

class ObservationTerm
{
  public:
  ObservationTerm(ObservationTermCfg cfg, std::function<std::vector<double>()> compute_fn)
      : cfg_(std::move(cfg)), compute_fn_(std::move(compute_fn))
  {
    if (cfg_.observation_dim <= 0 || cfg_.history_length <= 0 || !compute_fn_ ||
        cfg_.observation_dim > std::numeric_limits<int>::max() / cfg_.history_length)
      throw std::invalid_argument("Invalid observation dimensions or callback");
    validate_vector(cfg_.scale);
    validate_vector(cfg_.clip_min);
    validate_vector(cfg_.clip_max);
    if (cfg_.clip_min.empty() != cfg_.clip_max.empty())
      throw std::invalid_argument("Both observation clip bounds are required");
    for (size_t i = 0; i < cfg_.clip_min.size(); ++i)
      if (cfg_.clip_min[i] > cfg_.clip_max[i])
        throw std::invalid_argument("Observation clip minimum exceeds maximum");
    history_.resize(output_dim());
    current_observations_.resize(output_dim());
    reset();
  }
  virtual ~ObservationTerm() = default;

  void reset()
  {
    next_ = count_ = 0;
    std::fill(history_.begin(), history_.end(), 0.0);
    std::fill(current_observations_.begin(), current_observations_.end(), 0.0);
  }

  void compute()
  {
    auto values = compute_fn_();
    if (values.size() != static_cast<size_t>(cfg_.observation_dim))
      throw std::invalid_argument("Observation callback size mismatch");
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (!std::isfinite(values[i]))
        throw std::invalid_argument("Nonfinite observation");
      if (!cfg_.clip_min.empty())
        values[i] = std::clamp(values[i], cfg_.clip_min[i], cfg_.clip_max[i]);
      if (!cfg_.scale.empty())
        values[i] *= cfg_.scale[i];
      if (!std::isfinite(values[i]))
        throw std::invalid_argument("Nonfinite scaled observation");
    }
    const size_t dim = static_cast<size_t>(cfg_.observation_dim);
    std::copy(values.begin(), values.end(), history_.begin() + next_ * dim);
    next_ = (next_ + 1) % static_cast<size_t>(cfg_.history_length);
    count_ = std::min(count_ + 1, static_cast<size_t>(cfg_.history_length));
    std::fill(current_observations_.begin(), current_observations_.end(), 0.0);
    const size_t missing = static_cast<size_t>(cfg_.history_length) - count_;
    const size_t oldest = count_ == static_cast<size_t>(cfg_.history_length) ? next_ : 0;
    for (size_t h = 0; h < count_; ++h)
    {
      const size_t source = ((oldest + h) % static_cast<size_t>(cfg_.history_length)) * dim;
      std::copy_n(history_.begin() + source, dim, current_observations_.begin() + (missing + h) * dim);
    }
  }

  std::vector<double> observations() const
  {
    return current_observations_;
  }
  const std::vector<double>& observations_view() const
  {
    return current_observations_;
  }
  int observation_dim() const
  {
    return cfg_.observation_dim;
  }
  int output_dim() const
  {
    return cfg_.observation_dim * cfg_.history_length;
  }
  int history_length() const
  {
    return cfg_.history_length;
  }

  protected:
  ObservationTermCfg cfg_;

  private:
  void validate_vector(const std::vector<double>& values) const
  {
    if (!values.empty() && values.size() != static_cast<size_t>(cfg_.observation_dim))
      throw std::invalid_argument("Observation configuration vector size mismatch");
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument("Nonfinite observation configuration");
  }
  std::function<std::vector<double>()> compute_fn_;
  std::vector<double> history_, current_observations_;
  size_t next_{0}, count_{0};
};

struct ObservationManagerCfg
{
  bool term_order{true};
};

class ObservationManager
{
  public:
  explicit ObservationManager(ObservationManagerCfg cfg = {}) : cfg_(cfg)
  {
  }

  void add_term(std::unique_ptr<ObservationTerm> term)
  {
    if (!term)
      throw std::invalid_argument("Null observation term");
    if (term->output_dim() > std::numeric_limits<int>::max() - total_observation_dim())
      throw std::invalid_argument("Observation dimension overflow");
    terms_.push_back(std::move(term));
    observation_.resize(total_observation_dim());
    rebuild_layout();
  }

  void reset()
  {
    std::fill(observation_.begin(), observation_.end(), 0.0);
    for (auto& term : terms_) term->reset();
  }

  std::vector<double> observation() const
  {
    return observation_;
  }
  const std::vector<double>& observation_view() const
  {
    return observation_;
  }
  std::vector<double> compute()
  {
    return compute_view();
  }

  const std::vector<double>& compute_view()
  {
    for (auto& term : terms_) term->compute();
    for (size_t i = 0; i < layout_.size(); ++i)
      observation_[i] = terms_[layout_[i].first]->observations_view()[layout_[i].second];
    return observation_;
  }

  int total_observation_dim() const
  {
    int total = 0;
    for (const auto& term : terms_) total += term->output_dim();
    return total;
  }
  std::vector<int> observation_dim() const
  {
    std::vector<int> dimensions;
    for (const auto& term : terms_) dimensions.push_back(term->output_dim());
    return dimensions;
  }

  private:
  void rebuild_layout()
  {
    layout_.clear();
    if (cfg_.term_order)
    {
      for (size_t i = 0; i < terms_.size(); ++i)
        for (int j = 0; j < terms_[i]->output_dim(); ++j) layout_.emplace_back(i, j);
    }
    else
    {
      int max_history = 0;
      for (const auto& term : terms_) max_history = std::max(max_history, term->history_length());
      for (int h = 0; h < max_history; ++h)
        for (size_t i = 0; i < terms_.size(); ++i)
          if (h < terms_[i]->history_length())
            for (int j = 0; j < terms_[i]->observation_dim(); ++j)
              layout_.emplace_back(i, h * terms_[i]->observation_dim() + j);
    }
  }
  ObservationManagerCfg cfg_;
  std::vector<double> observation_;
  std::vector<std::unique_ptr<ObservationTerm>> terms_;
  std::vector<std::pair<size_t, size_t>> layout_;
};
}  // namespace onnx_policy::observation
