#pragma once

#include <vector>
#include <string>
#include <vector>

#include "onnxruntime_cxx_api.h"

namespace base_controllers
{

struct OnnxPolicyCfg
{
  std::string model_path;
  int input_size;
  int output_size;
  bool validate_model_io = false;
};

class OnnxPolicy
{
  // Implementation of inferencing an ONNX model using ONNX Runtime C++ CPU API
  public:
  OnnxPolicy(const std::string& model_path, int input_size, int output_size);
  OnnxPolicy(const OnnxPolicyCfg& config);

  std::vector<double> infer(const std::vector<double>& input_tensor_values);

  protected:
  OnnxPolicyCfg cfg_;

  std::shared_ptr<Ort::Env> env_ptr_ = nullptr;
  std::shared_ptr<Ort::Session> session_ptr_ = nullptr;
  std::shared_ptr<Ort::RunOptions> run_options_ptr_ = nullptr;
  std::string input_name_ = "";
  std::string output_name_ = "";
  std::array<int64_t, 2> input_shape_{1, 0};  // assuming 2D input tensor since MLP
  std::array<int64_t, 2> output_shape_{1, 0};
  std::shared_ptr<std::vector<float>> input_data_ptr_;
  std::shared_ptr<std::vector<float>> output_data_ptr_;
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
};

/*
MLP
*/
using OnnxMLP = OnnxPolicy;
using OnnxMLPCfg = OnnxPolicyCfg;

/*
RNN
*/
struct OnnxRNNPolicyCfg : public OnnxPolicyCfg
{
  int hidden_dim = 0;
  int hidden_layers = 0;
};

class OnnxRNN: public OnnxPolicy
{
  public:
    explicit OnnxRNN(const OnnxRNNPolicyCfg& cfg);

    std::vector<double> infer(const std::vector<double>& input_tensor_values);

    void reset(); // reset hidden states

  protected:
    void validate_model_io();

  protected:
    OnnxRNNPolicyCfg rnn_cfg_;

    std::vector<int64_t> hidden_shape_{};
    std::vector<float> hidden_in_data_;
    std::vector<float> hidden_out_data_;

    Ort::Value hidden_in_tensor_{nullptr};
    Ort::Value hidden_out_tensor_{nullptr};
};

}  // namespace base_controllers
