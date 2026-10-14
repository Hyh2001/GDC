#include <gtest/gtest.h>

#include <limits>

#include "onnx_policy/mdp/action/actions.hpp"
#include "onnx_policy/mdp/observation/observations.hpp"

using namespace onnx_policy;
namespace
{
std::unique_ptr<observation::ObservationTerm> term(int history, std::function<std::vector<double>()> callback)
{
  observation::ObservationTermCfg cfg;
  cfg.observation_dim = 1;
  cfg.history_length = history;
  return std::make_unique<observation::ObservationTerm>(cfg, callback);
}
}  // namespace

TEST(Observations, ClipBeforeScaleHistoryAndReset)
{
  observation::ObservationTermCfg cfg;
  cfg.observation_dim = 1;
  cfg.history_length = 3;
  cfg.clip_min = {-1};
  cfg.clip_max = {1};
  cfg.scale = {2};
  double input = 3;
  observation::ObservationTerm obs(cfg, [&] { return std::vector<double>{input}; });
  obs.compute();
  EXPECT_EQ(obs.observations(), (std::vector<double>{0, 0, 2}));
  input = -3;
  obs.compute();
  EXPECT_EQ(obs.observations(), (std::vector<double>{0, 2, -2}));
  input = .25;
  obs.compute();
  obs.compute();
  EXPECT_EQ(obs.observations(), (std::vector<double>{-2, .5, .5}));
  const auto* storage = obs.observations_view().data();
  obs.reset();
  EXPECT_EQ(obs.observations(), (std::vector<double>{0, 0, 0}));
  EXPECT_EQ(storage, obs.observations_view().data());
}

TEST(Observations, ConcatenationOrdersWithUnequalHistory)
{
  for (bool term_order : {true, false})
  {
    observation::ObservationManager manager({term_order});
    double a = 1, b = 10;
    manager.add_term(term(2, [&] { return std::vector<double>{a}; }));
    manager.add_term(term(1, [&] { return std::vector<double>{b}; }));
    EXPECT_EQ(manager.compute(), term_order ? (std::vector<double>{0, 1, 10}) : (std::vector<double>{0, 10, 1}));
    const auto* storage = manager.observation_view().data();
    a = 2;
    b = 20;
    EXPECT_EQ(manager.compute(), term_order ? (std::vector<double>{1, 2, 20}) : (std::vector<double>{1, 20, 2}));
    EXPECT_EQ(storage, manager.observation_view().data());
    EXPECT_EQ(manager.observation_dim(), (std::vector<int>{2, 1}));
    manager.reset();
    EXPECT_EQ(manager.observation(), (std::vector<double>{0, 0, 0}));
  }
}

TEST(Observations, RejectsMalformedConfigurationAndCallbacks)
{
  observation::ObservationTermCfg cfg;
  auto callback = [] { return std::vector<double>{1}; };
  EXPECT_THROW(observation::ObservationTerm(cfg, callback), std::invalid_argument);
  cfg.observation_dim = 1;
  cfg.history_length = 0;
  EXPECT_THROW(observation::ObservationTerm(cfg, callback), std::invalid_argument);
  cfg.history_length = 1;
  cfg.scale = {1, 2};
  EXPECT_THROW(observation::ObservationTerm(cfg, callback), std::invalid_argument);
  cfg.scale.clear();
  cfg.clip_min = {-1};
  EXPECT_THROW(observation::ObservationTerm(cfg, callback), std::invalid_argument);
  cfg.clip_max = {-2};
  EXPECT_THROW(observation::ObservationTerm(cfg, callback), std::invalid_argument);
  cfg.clip_min.clear();
  cfg.clip_max.clear();
  EXPECT_THROW(observation::ObservationTerm(cfg, {}), std::invalid_argument);
  observation::ObservationTerm bad_size(cfg, [] { return std::vector<double>{}; });
  EXPECT_THROW(bad_size.compute(), std::invalid_argument);
  observation::ObservationTerm bad_value(cfg,
                                         [] { return std::vector<double>{std::numeric_limits<double>::infinity()}; });
  EXPECT_THROW(bad_value.compute(), std::invalid_argument);
  observation::ObservationManager manager;
  EXPECT_THROW(manager.add_term(nullptr), std::invalid_argument);
}

TEST(Observations, HelpersValidateAndNormalize)
{
  EXPECT_EQ(observation::joint_values({2, 4}, {1, 2}), (std::vector<double>{1, 2}));
  EXPECT_THROW(observation::joint_values({1}, {}), std::invalid_argument);
  EXPECT_EQ(observation::projected_gravity(Eigen::Quaterniond(2, 0, 0, 0)), (std::vector<double>{0, 0, -1}));
  const auto rotated = observation::projected_gravity(
      Eigen::Quaterniond(Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitX())));
  EXPECT_NEAR(rotated[1], -1, 1e-12);
  EXPECT_NEAR(rotated[2], 0, 1e-12);
  EXPECT_THROW(observation::projected_gravity(Eigen::Quaterniond(0, 0, 0, 0)), std::invalid_argument);
  EXPECT_THROW(observation::projected_gravity(Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0)),
               std::invalid_argument);
}

TEST(Actions, PartitionTransformsAndReset)
{
  action::ActionManager manager;
  manager.add_term(
      std::make_unique<action::JointPositionActionTerm>(action::ActionTermCfg{"position", 2, {2, 3}, {10, 20}}));
  manager.add_term(std::make_unique<action::JointEffortActionTerm>(action::ActionTermCfg{"effort", 1, {4}, {100}}));
  manager.process_action({1, 2, 3});
  EXPECT_EQ(manager.processed_actions(), (std::vector<double>{12, 26, 12}));
  EXPECT_EQ(manager.raw_actions(), (std::vector<double>{1, 2, 3}));
  EXPECT_EQ(manager.action_dim(), (std::vector<int>{2, 1}));
  const auto* storage = manager.processed_actions_view().data();
  EXPECT_THROW(manager.process_action({1, 2}), std::invalid_argument);
  EXPECT_EQ(manager.raw_actions(), (std::vector<double>{1, 2, 3}));
  manager.reset();
  EXPECT_EQ(manager.raw_actions(), (std::vector<double>{0, 0, 0}));
  EXPECT_EQ(manager.processed_actions(), (std::vector<double>{0, 0, 0}));
  EXPECT_EQ(storage, manager.processed_actions_view().data());
  action::JointImpedanceActionTerm impedance({"", 1, {2}, {3}});
  impedance.process_actions({4});
  EXPECT_EQ(impedance.processed_actions(), (std::vector<double>{11}));
  impedance.reset();
  EXPECT_EQ(impedance.raw_actions(), (std::vector<double>{0}));
  EXPECT_EQ(impedance.processed_actions(), (std::vector<double>{0}));
  action::JointVelocityActionTerm velocity({"", 1, {2}, {3}});
  velocity.process_actions({4});
  EXPECT_EQ(velocity.processed_actions(), (std::vector<double>{11}));
}

TEST(Actions, RejectsInvalidDimensionsAndValues)
{
  EXPECT_THROW(action::JointPositionActionTerm((action::ActionTermCfg{"", 0, {}, {}})), std::invalid_argument);
  EXPECT_THROW(action::JointPositionActionTerm((action::ActionTermCfg{"", 1, {}, {0}})), std::invalid_argument);
  action::JointPositionActionTerm term({"", 1, {2}, {0}});
  EXPECT_THROW(term.process_actions({1, 2}), std::invalid_argument);
  EXPECT_THROW(term.process_actions({std::numeric_limits<double>::max()}), std::invalid_argument);
  action::ActionManager manager;
  EXPECT_THROW(manager.add_term(nullptr), std::invalid_argument);
  manager.add_term(std::make_unique<action::JointPositionActionTerm>(action::ActionTermCfg{"", 1, {1}, {0}}));
  EXPECT_THROW(manager.process_action({std::numeric_limits<double>::quiet_NaN()}), std::invalid_argument);
}
