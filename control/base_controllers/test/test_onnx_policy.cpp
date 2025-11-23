#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include "base_controllers/onnx_policy.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_onnx_model>" << std::endl;
        return -1;
    }

    std::string model_path = argv[1];
    
    try {
        // Initialize the ONNX policy with specific parameters
        std::cout << "Loading ONNX model from: " << model_path << std::endl;
        
        // Adjust these parameters based on your model
        int input_size = 3;   // Change this to match your model's input size
        int output_size = 1;  // Change this to match your model's output size
        
        base_controllers::OnnxPolicy policy(model_path, input_size, output_size);
        std::cout << "Model loaded successfully!" << std::endl;

        // Generate test input data (using double to match your implementation)
        std::vector<double> test_input(input_size);
        
        // Fill with random values between -1 and 1
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(-1.0, 1.0);  // Changed to double
        
        for (int i = 0; i < input_size; ++i) {
            test_input[i] = dis(gen);
        }

        std::cout << "\nTest input (" << input_size << " values): ";
        for (size_t i = 0; i < test_input.size(); ++i) {
            std::cout << test_input[i];
            if (i < test_input.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;

        // Run inference
        std::cout << "\nRunning inference..." << std::endl;
        
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<double> output = policy.infer(test_input);  // Changed to double
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Inference completed in " << duration.count() << " microseconds" << std::endl;

        // Print output
        std::cout << "\nModel output (" << output.size() << " values): ";
        for (size_t i = 0; i < output.size(); ++i) {
            std::cout << output[i];
            if (i < output.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;

        // Run multiple inferences to test performance
        std::cout << "\nRunning performance test (1000 inferences)..." << std::endl;
        
        auto perf_start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 1000; ++i) {
            // Slightly modify input for each run
            for (auto& val : test_input) {
                val += 0.0001 * i;  // Changed to double
            }
            policy.infer(test_input);
        }
        auto perf_end = std::chrono::high_resolution_clock::now();
        
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(perf_end - perf_start);
        std::cout << "1000 inferences completed in " << total_duration.count() << " ms" << std::endl;
        std::cout << "Average time per inference: " << total_duration.count() / 1000.0 << " ms" << std::endl;

        // Test with different input patterns
        std::cout << "\nTesting with different input patterns..." << std::endl;
        
        // Test with zeros
        std::fill(test_input.begin(), test_input.end(), 0.0);  // Changed to double
        auto zero_output = policy.infer(test_input);
        std::cout << "Zero input -> Output[0]: " << zero_output[0] << std::endl;
        
        // Test with ones
        std::fill(test_input.begin(), test_input.end(), 1.0);  // Changed to double
        auto ones_output = policy.infer(test_input);
        std::cout << "Ones input -> Output[0]: " << ones_output[0] << std::endl;
        
        // Test with negative ones
        std::fill(test_input.begin(), test_input.end(), -1.0);  // Changed to double
        auto neg_ones_output = policy.infer(test_input);
        std::cout << "Negative ones input -> Output[0]: " << neg_ones_output[0] << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "\nTest completed successfully!" << std::endl;
    return 0;
}