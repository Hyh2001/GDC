#pragma once
#ifndef OBSERVATION_MANAGER_HPP_
#define OBSERVATION_MANAGER_HPP_

#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include <functional>
#include <stdexcept>
#include <algorithm>

namespace observation
{

struct ObservationTermCfg
{
  // compute -> clip -> scale -> history (always concatenate)
  std::string name{};
  int observation_dim{0}; // for sanity check
  std::vector<double> scale{};
  std::vector<double> clip_min{};
  std::vector<double> clip_max{};
  int history_length{1};
};

class ObservationTerm
{
public:
  ObservationTerm(
      ObservationTermCfg cfg,
      std::function<std::vector<double>()> compute_fn)
    : cfg_(std::move(cfg)), compute_fn_(std::move(compute_fn))
  {
    if (cfg_.history_length < 1)
    {
      throw std::invalid_argument("history_length must be at least 1.");
    }
    reset();
  }
  virtual ~ObservationTerm() = default;

  void reset()
  {
    history_.clear();
    current_observations_.assign(output_dim(), 0.0);
  }

  void compute()
  {
    std::vector<double> obs = compute_fn_();

    if (obs.size() != static_cast<size_t>(cfg_.observation_dim)) {
      throw std::runtime_error("ObservationTerm: observation size mismatch.");
    }

    apply_clip(obs);
    apply_scale(obs);

    push_history(obs);
    current_observations_ = build_output_from_history();
  }

  std::vector<double> observations() const
  {
    return current_observations_;
  }

  int observation_dim() const
  {
    return cfg_.observation_dim;
  }

  int output_dim() const
  {
    return cfg_.observation_dim * std::max(1, cfg_.history_length);
  }

  int history_length() const
  {
    return cfg_.history_length;
  }

protected:
  void apply_scale(std::vector<double>& obs)
  {
    if (cfg_.scale.empty()) {
      return;
    }
    if (cfg_.scale.size() != obs.size()) {
      throw std::runtime_error("ObservationTerm: scale size mismatch.");
    }
    for (size_t i = 0; i < obs.size(); ++i) {
      obs[i] *= cfg_.scale[i];
    }
  }

  void apply_clip(std::vector<double>& obs)
  {
    if (cfg_.clip_min.empty() || cfg_.clip_max.empty()) {
      return;
    }
    if (cfg_.clip_min.size() != obs.size() || cfg_.clip_max.size() != obs.size()) {
      throw std::runtime_error("ObservationTerm: clip bound size mismatch.");
    }
    for (size_t i = 0; i < obs.size(); ++i) {
      obs[i] = std::min(std::max(obs[i], cfg_.clip_min[i]), cfg_.clip_max[i]);
    }
  }

  void push_history(const std::vector<double>& obs)
  {
    history_.push_back(obs);
    while (history_.size() > static_cast<size_t>(cfg_.history_length)) {
      history_.erase(history_.begin());
    }
  }

  std::vector<double> build_output_from_history() const
  {
    std::vector<double> out;
    out.reserve(output_dim());

    // fill in missing history
    const int missing = cfg_.history_length - static_cast<int>(history_.size());
    for (int i = 0; i < missing; ++i) {
      out.insert(out.end(), cfg_.observation_dim, 0.0);
    }

    for (const auto& h : history_) {
      out.insert(out.end(), h.begin(), h.end());
    }

    return out;
  }

  ObservationTermCfg cfg_;
  std::function<std::vector<double>()> compute_fn_;
  std::vector<std::vector<double>> history_;
  std::vector<double> current_observations_;
};

struct ObservationManagerCfg
{
  bool term_order{true}; // default history use term order, false lead to time order
};


class ObservationManager
{
public:
  explicit ObservationManager(ObservationManagerCfg cfg = {})
    : cfg_(std::move(cfg))
  {}

  void add_term(std::unique_ptr<ObservationTerm> term)
  {
    terms_.push_back(std::move(term));
    observation_.assign(total_observation_dim(), 0.0);
  }

  void reset()
  {
    observation_.assign(total_observation_dim(), 0.0);
    for (auto& term : terms_)
    {
      term->reset();
    }
  }

  std::vector<double> observation() const
  {
    return observation_;
  }

  std::vector<double> compute()
  {
    std::vector<std::vector<double>> term_outputs;
    term_outputs.reserve(terms_.size());

    for (auto& term : terms_) {
      term->compute();
      term_outputs.push_back(term->observations());
    }

    if (cfg_.term_order) {
      observation_ =  concatenate_term_order(term_outputs);
    } else {
      observation_ =  concatenate_time_order(term_outputs);
    }
    return observation_;
  }

  int total_observation_dim() const
  {
    const auto dims = observation_dim();
    return std::accumulate(dims.begin(), dims.end(), 0);
  }

  std::vector<int> observation_dim() const
  {
    std::vector<int> dims;
    dims.reserve(terms_.size());
    for (const auto& term : terms_)
    {
      dims.push_back(term->output_dim());
    }
    return dims;
  }

private:
  std::vector<double> concatenate_term_order(
    const std::vector<std::vector<double>>& term_outputs) const
  {
    std::vector<double> out;
    out.reserve(total_observation_dim());

    for (const auto& term_obs : term_outputs) {
      out.insert(out.end(), term_obs.begin(), term_obs.end());
    }

    return out;
  }

  std::vector<double> concatenate_time_order(
    const std::vector<std::vector<double>>& term_outputs) const
  {
    std::vector<double> out;
    out.reserve(total_observation_dim());

    int max_history = 0;
    for (const auto& term : terms_) {
      max_history = std::max(max_history, term->history_length());
    }

    for (int h = 0; h < max_history; ++h) {
      for (size_t term_idx = 0; term_idx < terms_.size(); ++term_idx) {
        const int dim = terms_[term_idx]->observation_dim();
        const int hist = terms_[term_idx]->history_length();

        if (h >= hist) {
          continue;
        }

        const int start = h * dim;
        const int end = start + dim;

        out.insert(
            out.end(),
            term_outputs[term_idx].begin() + start,
            term_outputs[term_idx].begin() + end);
      }
    }

    return out;
  }


  ObservationManagerCfg cfg_;
  std::vector<double> observation_;
  std::vector<std::unique_ptr<ObservationTerm>> terms_;
};

}  // namespace observation

#endif  // OBSERVATION_MANAGER_HPP_
