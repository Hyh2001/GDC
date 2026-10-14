#pragma once

#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace onnx_policy
{
struct OnnxPolicyCfg
{
  std::string model_path;
  int input_size{0};
  int output_size{0};
  // Retained for source familiarity; essential model checks are always performed.
  bool validate_model_io{false};
};

class OnnxPolicy
{
  public:
  OnnxPolicy(const std::string& model_path, int input_size, int output_size);
  explicit OnnxPolicy(const OnnxPolicyCfg& config);
  virtual ~OnnxPolicy() = default;
  OnnxPolicy(const OnnxPolicy&) = delete;
  OnnxPolicy& operator=(const OnnxPolicy&) = delete;

  virtual std::vector<double> infer(const std::vector<double>& input);
  virtual void infer_into(const std::vector<double>& input, std::vector<double>& output);
  virtual void reset()
  {
  }

  protected:
  OnnxPolicy(const OnnxPolicyCfg& config, bool recurrent);
  void validate_tensor(bool input, size_t index, const std::vector<int64_t>& shape) const;
  void bind_names();

  OnnxPolicyCfg cfg_;
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  Ort::RunOptions run_options_;
  std::vector<float> input_data_, output_data_;
  std::vector<std::string> input_names_, output_names_;
  std::vector<const char*> input_name_ptrs_, output_name_ptrs_;
  std::vector<Ort::Value> input_tensors_, output_tensors_;
};

using OnnxMLP = OnnxPolicy;
using OnnxMLPCfg = OnnxPolicyCfg;

struct OnnxRNNPolicyCfg : OnnxPolicyCfg
{
  int hidden_dim{0};
  int hidden_layers{0};
};

class OnnxRNN : public OnnxPolicy
{
  public:
  explicit OnnxRNN(const OnnxRNNPolicyCfg& config);
  void infer_into(const std::vector<double>& input, std::vector<double>& output) override;
  void reset() override;

  private:
  std::vector<float> hidden_in_data_, hidden_out_data_;
};
}  // namespace onnx_policy
