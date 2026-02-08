#pragma once
#include <vector>

#include "onnxruntime_cxx_api.h"

namespace base_controllers
{

class OnnxPolicy
{
  // Implementation of inferencing an ONNX model using ONNX Runtime C++ CPU API
  public:
  OnnxPolicy(const std::string& model_path, int input_size, int output_size);

  void configure();  // configure the runtime session

  std::vector<double> infer(const std::vector<double>& input_tensor_values);

  void cleanup();  // cleanup resources

  protected:
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

}  // namespace base_controllers
