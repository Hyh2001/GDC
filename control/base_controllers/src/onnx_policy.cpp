#include "base_controllers/onnx_policy.hpp"

namespace base_controllers
{

OnnxPolicy::OnnxPolicy(const std::string& model_path, int input_size, int output_size)
{
  input_shape_[1] = input_size;
  output_shape_[1] = output_size;
  input_data_ptr_ = std::make_shared<std::vector<float>>();
  output_data_ptr_ = std::make_shared<std::vector<float>>();
  input_data_ptr_->resize(input_size);
  output_data_ptr_->resize(output_size);
  // Initialize ONNX Runtime environment
  env_ptr_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "OnnxPolicy");
  // Set basic session options
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  // Create session
  session_ptr_ = std::make_unique<Ort::Session>(*env_ptr_, model_path.c_str(), session_options);
  // Create run options
  run_options_ptr_ = std::make_shared<Ort::RunOptions>();

  // Create data tensors
  input_tensor_ = Ort::Value::CreateTensor<float>(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
                                                  input_data_ptr_->data(), input_data_ptr_->size(), input_shape_.data(),
                                                  input_shape_.size());
  output_tensor_ = Ort::Value::CreateTensor<float>(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
                                                   output_data_ptr_->data(), output_data_ptr_->size(),
                                                   output_shape_.data(), output_shape_.size());
  // names
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::AllocatedStringPtr input_name_ptr = session_ptr_->GetInputNameAllocated(0, allocator);
  input_name_ = std::string(input_name_ptr.get());
  Ort::AllocatedStringPtr output_name_ptr = session_ptr_->GetOutputNameAllocated(0, allocator);
  output_name_ = std::string(output_name_ptr.get());
}

OnnxPolicy::OnnxPolicy(const OnnxPolicyCfg& config) : cfg_(config)
{
  // Delegate to the other constructor
  *this = OnnxPolicy(config.model_path, config.input_size, config.output_size);
}

std::vector<double> OnnxPolicy::infer(const std::vector<double>& input_tensor_values)
{
  for (size_t i = 0; i < input_tensor_values.size(); ++i)
  {
    (*input_data_ptr_)[i] = static_cast<float>(input_tensor_values[i]);
  }
  const char* input_names[] = {input_name_.c_str()};
  const char* output_names[] = {output_name_.c_str()};
  session_ptr_->Run(*run_options_ptr_, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);

  std::vector<double> result(output_data_ptr_->size());
  for (size_t i = 0; i < output_data_ptr_->size(); ++i)
  {
    result[i] = static_cast<double>((*output_data_ptr_)[i]);
  }

  return result;
}

OnnxRNN::OnnxRNN(const OnnxRNNPolicyCfg& cfg) : OnnxPolicy(cfg), rnn_cfg_(cfg)
{
  hidden_shape_ = {rnn_cfg_.hidden_layers, 1, rnn_cfg_.hidden_dim};

  const size_t hidden_size =
      static_cast<size_t>(rnn_cfg_.hidden_layers) *
      static_cast<size_t>(rnn_cfg_.hidden_dim);

  hidden_in_data_.resize(hidden_size, 0.0f);
  hidden_out_data_.resize(hidden_size, 0.0f);

  hidden_in_tensor_ = Ort::Value::CreateTensor<float>(
      Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
      hidden_in_data_.data(),
      hidden_in_data_.size(),
      hidden_shape_.data(),
      hidden_shape_.size());

  hidden_out_tensor_ = Ort::Value::CreateTensor<float>(
      Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
      hidden_out_data_.data(),
      hidden_out_data_.size(),
      hidden_shape_.data(),
      hidden_shape_.size());

  if (rnn_cfg_.validate_model_io)
  {
    validate_model_io();
  }
}

std::vector<double> OnnxRNN::infer(const std::vector<double>& input_tensor_values)
{
  if (input_tensor_values.size() != input_data_ptr_->size())
  {
    throw std::runtime_error("OnnxRNN::infer input size mismatch");
  }

  for (size_t i = 0; i < input_tensor_values.size(); ++i)
  {
    (*input_data_ptr_)[i] = static_cast<float>(input_tensor_values[i]);
  }

  const char* input_names[] = {input_name_.c_str(), "h_in"};
  const char* output_names[] = {output_name_.c_str(), "h_out"};

  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
  Ort::Value input_tensors[] = {
      Ort::Value::CreateTensor<float>(
          memory_info,
          input_data_ptr_->data(),
          input_data_ptr_->size(),
          input_shape_.data(),
          input_shape_.size()),
      Ort::Value::CreateTensor<float>(
          memory_info,
          hidden_in_data_.data(),
          hidden_in_data_.size(),
          hidden_shape_.data(),
          hidden_shape_.size())
  };

  Ort::Value output_tensors[] = {
      Ort::Value::CreateTensor<float>(
          memory_info,
          output_data_ptr_->data(),
          output_data_ptr_->size(),
          output_shape_.data(),
          output_shape_.size()),
      Ort::Value::CreateTensor<float>(
          memory_info,
          hidden_out_data_.data(),
          hidden_out_data_.size(),
          hidden_shape_.data(),
          hidden_shape_.size())
  };

  session_ptr_->Run(
      *run_options_ptr_,
      input_names,
      input_tensors,
      2,
      output_names,
      output_tensors,
      2);

  hidden_in_data_ = hidden_out_data_;

  std::vector<double> result(output_data_ptr_->size());
  for (size_t i = 0; i < output_data_ptr_->size(); ++i)
  {
    result[i] = static_cast<double>((*output_data_ptr_)[i]);
  }

  return result;
}

void OnnxRNN::reset()
{
  std::fill(hidden_in_data_.begin(), hidden_in_data_.end(), 0.0f);
  std::fill(hidden_out_data_.begin(), hidden_out_data_.end(), 0.0f);

  hidden_in_tensor_ = Ort::Value::CreateTensor<float>(
      Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
      hidden_in_data_.data(),
      hidden_in_data_.size(),
      hidden_shape_.data(),
      hidden_shape_.size());

  hidden_out_tensor_ = Ort::Value::CreateTensor<float>(
      Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
      hidden_out_data_.data(),
      hidden_out_data_.size(),
      hidden_shape_.data(),
      hidden_shape_.size());
}

void OnnxRNN::validate_model_io()
{
  // TODO: determine what to validate?
}

}  // namespace base_controllers
