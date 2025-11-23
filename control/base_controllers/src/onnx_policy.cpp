#include "base_controllers/onnx_policy.hpp"

namespace base_controllers{

OnnxPolicy::OnnxPolicy(const std::string& model_path, 
    int input_size, int output_size) {
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
    input_tensor_ = Ort::Value::CreateTensor<float>(
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
        input_data_ptr_->data(), 
        input_data_ptr_->size(), 
        input_shape_.data(),
        input_shape_.size()); 
    output_tensor_ = Ort::Value::CreateTensor<float>(
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
        output_data_ptr_->data(), 
        output_data_ptr_->size(), 
        output_shape_.data(),
        output_shape_.size());
    // names
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::AllocatedStringPtr input_name_ptr = 
        session_ptr_->GetInputNameAllocated(0, allocator);
    input_name_ = std::string(input_name_ptr.get());
    Ort::AllocatedStringPtr output_name_ptr = 
        session_ptr_->GetOutputNameAllocated(0, allocator);
    output_name_ = std::string(output_name_ptr.get());
}

std::vector<double> OnnxPolicy::infer(
    const std::vector<double> &input_tensor_values)
{
    for (size_t i = 0; i < input_tensor_values.size(); ++i) {
        (*input_data_ptr_)[i] = static_cast<float>(input_tensor_values[i]);
    }
    const char* input_names[] = {input_name_.c_str()};
    const char* output_names[] = {output_name_.c_str()};
    session_ptr_->Run(
        *run_options_ptr_,
        input_names, &input_tensor_, 1,
        output_names, &output_tensor_, 1);
    
        std::vector<double> result(output_data_ptr_->size());
    for (size_t i = 0; i < output_data_ptr_->size(); ++i) {
        result[i] = static_cast<double>((*output_data_ptr_)[i]);
    }
    
    return result;
}

} // namespace base_controllers