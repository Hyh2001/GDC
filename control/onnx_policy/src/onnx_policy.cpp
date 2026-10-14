#include "onnx_policy/onnx_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace onnx_policy
{
namespace
{
size_t tensor_index(const Ort::Session& session, bool input, const std::string& name)
{
  Ort::AllocatorWithDefaultOptions allocator;
  const size_t count = input ? session.GetInputCount() : session.GetOutputCount();
  for (size_t i = 0; i < count; ++i)
  {
    const auto candidate =
        input ? session.GetInputNameAllocated(i, allocator) : session.GetOutputNameAllocated(i, allocator);
    if (name == candidate.get())
      return i;
  }
  throw std::invalid_argument("Missing model tensor: " + name);
}

std::string primary_name(const Ort::Session& session, bool input, const std::string& hidden)
{
  Ort::AllocatorWithDefaultOptions allocator;
  const size_t count = input ? session.GetInputCount() : session.GetOutputCount();
  for (size_t i = 0; i < count; ++i)
  {
    const auto name =
        input ? session.GetInputNameAllocated(i, allocator) : session.GetOutputNameAllocated(i, allocator);
    if (hidden != name.get())
      return name.get();
  }
  throw std::invalid_argument("Missing primary model tensor");
}
}  // namespace

OnnxPolicy::OnnxPolicy(const std::string& path, int input_size, int output_size)
    : OnnxPolicy(OnnxPolicyCfg{path, input_size, output_size, false})
{
}

OnnxPolicy::OnnxPolicy(const OnnxPolicyCfg& config) : OnnxPolicy(config, false)
{
}

OnnxPolicy::OnnxPolicy(const OnnxPolicyCfg& config, bool recurrent)
    : cfg_(config), env_(ORT_LOGGING_LEVEL_WARNING, "onnx_policy")
{
  if (cfg_.input_size <= 0 || cfg_.output_size <= 0 || cfg_.model_path.empty())
    throw std::invalid_argument("Model path and positive input/output dimensions are required");
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  session_ = std::make_unique<Ort::Session>(env_, cfg_.model_path.c_str(), options);
  const size_t count = recurrent ? 2 : 1;
  if (session_->GetInputCount() != count || session_->GetOutputCount() != count)
    throw std::invalid_argument("Unsupported model input/output count");
  if (recurrent)
  {
    tensor_index(*session_, true, "h_in");
    tensor_index(*session_, false, "h_out");
  }
  input_names_.push_back(primary_name(*session_, true, recurrent ? "h_in" : ""));
  output_names_.push_back(primary_name(*session_, false, recurrent ? "h_out" : ""));
  const std::vector<int64_t> input_shape{1, cfg_.input_size};
  const std::vector<int64_t> output_shape{1, cfg_.output_size};
  validate_tensor(true, tensor_index(*session_, true, input_names_[0]), input_shape);
  validate_tensor(false, tensor_index(*session_, false, output_names_[0]), output_shape);
  input_data_.resize(cfg_.input_size);
  output_data_.resize(cfg_.output_size);
  const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  input_tensors_.push_back(Ort::Value::CreateTensor<float>(memory, input_data_.data(), input_data_.size(),
                                                           input_shape.data(), input_shape.size()));
  output_tensors_.push_back(Ort::Value::CreateTensor<float>(memory, output_data_.data(), output_data_.size(),
                                                            output_shape.data(), output_shape.size()));
  bind_names();
}

void OnnxPolicy::validate_tensor(bool input, size_t index, const std::vector<int64_t>& expected) const
{
  const auto info = input ? session_->GetInputTypeInfo(index) : session_->GetOutputTypeInfo(index);
  if (info.GetONNXType() != ONNX_TYPE_TENSOR)
    throw std::invalid_argument("Policy IO must be tensors");
  const auto tensor = info.GetTensorTypeAndShapeInfo();
  if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    throw std::invalid_argument("Policy IO must be float32");
  const auto actual = tensor.GetShape();
  if (actual.size() != expected.size())
    throw std::invalid_argument("Policy tensor rank mismatch");
  for (size_t i = 0; i < actual.size(); ++i)
    if (actual[i] != -1 && actual[i] != expected[i])
      throw std::invalid_argument("Policy tensor dimension mismatch");
}

void OnnxPolicy::bind_names()
{
  input_name_ptrs_.clear();
  output_name_ptrs_.clear();
  for (const auto& name : input_names_) input_name_ptrs_.push_back(name.c_str());
  for (const auto& name : output_names_) output_name_ptrs_.push_back(name.c_str());
}

std::vector<double> OnnxPolicy::infer(const std::vector<double>& input)
{
  std::vector<double> output(output_data_.size());
  infer_into(input, output);
  return output;
}

void OnnxPolicy::infer_into(const std::vector<double>& input, std::vector<double>& output)
{
  if (input.size() != input_data_.size() || output.size() != output_data_.size())
    throw std::invalid_argument("Policy inference buffer size mismatch");
  for (size_t i = 0; i < input.size(); ++i)
  {
    if (!std::isfinite(input[i]) || std::abs(input[i]) > std::numeric_limits<float>::max())
      throw std::invalid_argument("Policy input must be finite float32 values");
    input_data_[i] = static_cast<float>(input[i]);
  }
  session_->Run(run_options_, input_name_ptrs_.data(), input_tensors_.data(), input_tensors_.size(),
                output_name_ptrs_.data(), output_tensors_.data(), output_tensors_.size());
  if (!std::all_of(output_data_.begin(), output_data_.end(), [](float value) { return std::isfinite(value); }))
    throw std::runtime_error("Policy output is nonfinite");
  std::copy(output_data_.begin(), output_data_.end(), output.begin());
}

OnnxRNN::OnnxRNN(const OnnxRNNPolicyCfg& config) : OnnxPolicy(config, true)
{
  if (config.hidden_layers <= 0 || config.hidden_dim <= 0)
    throw std::invalid_argument("Positive recurrent dimensions are required");
  const std::vector<int64_t> shape{config.hidden_layers, 1, config.hidden_dim};
  validate_tensor(true, tensor_index(*session_, true, "h_in"), shape);
  validate_tensor(false, tensor_index(*session_, false, "h_out"), shape);
  const size_t size = static_cast<size_t>(config.hidden_layers) * static_cast<size_t>(config.hidden_dim);
  hidden_in_data_.resize(size);
  hidden_out_data_.resize(size);
  const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  input_tensors_.push_back(
      Ort::Value::CreateTensor<float>(memory, hidden_in_data_.data(), size, shape.data(), shape.size()));
  output_tensors_.push_back(
      Ort::Value::CreateTensor<float>(memory, hidden_out_data_.data(), size, shape.data(), shape.size()));
  input_names_.push_back("h_in");
  output_names_.push_back("h_out");
  bind_names();
}

void OnnxRNN::infer_into(const std::vector<double>& input, std::vector<double>& output)
{
  OnnxPolicy::infer_into(input, output);
  if (!std::all_of(hidden_out_data_.begin(), hidden_out_data_.end(), [](float value) { return std::isfinite(value); }))
    throw std::runtime_error("Policy hidden state is nonfinite");
  std::copy(hidden_out_data_.begin(), hidden_out_data_.end(), hidden_in_data_.begin());
}

void OnnxRNN::reset()
{
  std::fill(hidden_in_data_.begin(), hidden_in_data_.end(), 0.0f);
  std::fill(hidden_out_data_.begin(), hidden_out_data_.end(), 0.0f);
}
}  // namespace onnx_policy
