#include <gtest/gtest.h>

#include <limits>

#include "onnx_policy/onnx_policy.hpp"

using namespace onnx_policy;
namespace
{
std::string fixture(const char* name)
{
  return std::string(FIXTURE_DIR) + "/" + name;
}
}  // namespace

TEST(Runtime, MlpAndCallerOwnedBuffer)
{
  OnnxPolicy policy(OnnxPolicyCfg{fixture("mlp.onnx"), 3, 2, true});
  EXPECT_EQ(policy.infer({1, 2, 3}), (std::vector<double>{3, 3}));
  std::vector<double> output(2);
  const auto* storage = output.data();
  policy.infer_into({4, 5, 6}, output);
  EXPECT_EQ(output, (std::vector<double>{9, 6}));
  EXPECT_EQ(storage, output.data());
  policy.reset();
  EXPECT_EQ(policy.infer({1, 2, 3}), (std::vector<double>{3, 3}));
}

TEST(Runtime, RejectsInvalidBuffersAndValues)
{
  OnnxPolicy policy(fixture("mlp.onnx"), 3, 2);
  EXPECT_THROW(policy.infer({1, 2}), std::invalid_argument);
  EXPECT_THROW(policy.infer({1, 2, 3, 4}), std::invalid_argument);
  std::vector<double> output(1);
  EXPECT_THROW(policy.infer_into({1, 2, 3}, output), std::invalid_argument);
  EXPECT_THROW(policy.infer({0, 0, std::numeric_limits<double>::quiet_NaN()}), std::invalid_argument);
  EXPECT_THROW(policy.infer({0, 0, std::numeric_limits<double>::max()}), std::invalid_argument);
  OnnxPolicy overflow(fixture("overflow.onnx"), 2, 2);
  output.assign(2, 42);
  EXPECT_THROW(overflow.infer_into({1e38, 1e38}, output), std::runtime_error);
  EXPECT_EQ(output, (std::vector<double>{42, 42}));
}

TEST(Runtime, ValidatesModelContractsEvenWithFlagDisabled)
{
  EXPECT_THROW(OnnxPolicy(fixture("mlp.onnx"), 0, 2), std::invalid_argument);
  EXPECT_THROW(OnnxPolicy(fixture("mlp.onnx"), 3, 1), std::invalid_argument);
  EXPECT_THROW(OnnxPolicy(fixture("mlp.onnx"), 2, 2), std::invalid_argument);
  EXPECT_THROW(OnnxPolicy(fixture("double.onnx"), 2, 2), std::invalid_argument);
  EXPECT_THROW(OnnxPolicy(fixture("rank.onnx"), 2, 2), std::invalid_argument);
  EXPECT_THROW(OnnxPolicy(fixture("extra.onnx"), 2, 2), std::invalid_argument);
  EXPECT_THROW(OnnxPolicy(fixture("rnn.onnx"), 2, 2), std::invalid_argument);
  EXPECT_THROW(OnnxPolicy(fixture("missing.onnx"), 2, 2), Ort::Exception);
  OnnxPolicy dynamic(fixture("dynamic.onnx"), 2, 2);
  EXPECT_EQ(dynamic.infer({2, 3}), (std::vector<double>{2, 3}));
}

TEST(Runtime, RecurrentDispatchMetadataOrderAndReset)
{
  OnnxRNNPolicyCfg cfg;
  cfg.model_path = fixture("rnn.onnx");
  cfg.input_size = cfg.output_size = cfg.hidden_dim = 2;
  cfg.hidden_layers = 1;
  std::unique_ptr<OnnxPolicy> policy = std::make_unique<OnnxRNN>(cfg);
  EXPECT_EQ(policy->infer({1, 2}), (std::vector<double>{1, 2}));
  std::vector<double> output(2);
  policy->infer_into({3, 4}, output);
  EXPECT_EQ(output, (std::vector<double>{5, 7}));
  policy->reset();
  EXPECT_EQ(policy->infer({1, 2}), (std::vector<double>{1, 2}));
  cfg.hidden_dim = 3;
  EXPECT_THROW(OnnxRNN{cfg}, std::invalid_argument);
  cfg.hidden_dim = 0;
  EXPECT_THROW(OnnxRNN{cfg}, std::invalid_argument);
  cfg.hidden_dim = 2;
  cfg.model_path = fixture("extra.onnx");
  EXPECT_THROW(OnnxRNN{cfg}, std::invalid_argument);
}
