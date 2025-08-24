#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <memory>

#include "common/INoiseGenerator.hpp"
#include "models/Unicycle.hpp"
#include "sim/GroundTruthSimulator.hpp"

using namespace testing;

template <int M> class MockNoiseGenerator : public common::INoiseGenerator<M> {
public:
  MOCK_METHOD((Eigen::Matrix<double, M, 1>), generate,
              ((const Eigen::Matrix<double, M, M> &covariance)),
              (const, override));
};

TEST(GroundTruthTest, AdvanceState_AddsCorrectProcessNoise) {
  static constexpr int state_size = 3;
  static constexpr int control_size = 2;
  double dt = 2;
  double epsilon = 1e-6;

  Eigen::Vector3d initial_state;
  initial_state << 0.0, 0.0, M_PI / 2.0;

  typename sim::GroundTruthSimulator<state_size, control_size>::ControlInput U;
  U << 1.0, 0.0;

  Eigen::Vector3d expected_noise;
  expected_noise << 0.1, 0.2, 0.3;

  MockNoiseGenerator<3> mock_noise_generator;

  EXPECT_CALL(mock_noise_generator, generate(A<const Eigen::Matrix3d &>()))
      .WillOnce(Return(expected_noise));

  typename sim::GroundTruthSimulator<state_size,
                                     control_size>::ProcessNoiseMatrix Q;
  Q << 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.005;

  models::Unicycle<state_size, control_size> unicycle_model;
  auto expected_perfect_state = unicycle_model.getNextState(initial_state, U, dt);
  auto expected_final_noisy_state = expected_perfect_state + expected_noise;

  auto unicycle_ptr =
      std::make_unique<models::Unicycle<state_size, control_size>>();
  sim::GroundTruthSimulator<state_size, control_size> ground_truth_sim(
      std::move(unicycle_ptr), initial_state, Q, mock_noise_generator);

  ground_truth_sim.advanceState(U, dt);

  ASSERT_NEAR(ground_truth_sim.getNoisyState()(0),
              expected_final_noisy_state(0), epsilon);
  ASSERT_NEAR(ground_truth_sim.getNoisyState()(1),
              expected_final_noisy_state(1), epsilon);
  ASSERT_NEAR(ground_truth_sim.getNoisyState()(2),
              expected_final_noisy_state(2), epsilon);

  ASSERT_NEAR(ground_truth_sim.getPerfectState()(0), expected_perfect_state(0),
              epsilon);
  ASSERT_NEAR(ground_truth_sim.getPerfectState()(1), expected_perfect_state(1),
              epsilon);
  ASSERT_NEAR(ground_truth_sim.getPerfectState()(2), expected_perfect_state(2),
              epsilon);
}