#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>

#include "common/EkfTraits.hpp"
#include "common/common.hpp"
#include "models/Unicycle.hpp"

static constexpr int StateSize = 3;
static constexpr int ControlSize = 2;

using StateVector = common::VehicleTypes<StateSize, ControlSize>::StateVector;
using ControlInput =
    common::VehicleTypes<StateSize, ControlSize>::ControlVector;
using StateMatrix = common::VehicleTypes<StateSize, ControlSize>::StateMatrix;

class UnicycleModelTest : public testing::Test {
protected:
  models::Unicycle<StateSize, ControlSize> model;
  double dt = 2.0;
  double epsilon = 1e-4;
};

TEST_F(UnicycleModelTest, GetDimensionsReturnsCorrectValue) {
  ASSERT_EQ(static_cast<int>(
                models::Unicycle<StateSize,
                                 ControlSize>::StateVector::SizeAtCompileTime),
            3);

  ASSERT_EQ(
      static_cast<int>(
          models::Unicycle<StateSize,
                           ControlSize>::ControlVector::SizeAtCompileTime),
      2);
};

TEST_F(UnicycleModelTest, ComputeJacobian_StraightMotion) {
  StateVector X;
  X << 1.0, 2.0, M_PI / 2.0; // theta = 1.5708

  ControlInput U;
  U << 10.0, 0.0; // v = 10, omega = 0

  StateMatrix F = model.computeFt(X, U, dt);

  double expected_F_0_2 =
      -U(0) * dt * std::sin(X(2)); // -10 * 2 * sin(1.5708) = -20.0
  double expected_F_1_2 =
      U(0) * dt * std::cos(X(2)); // 10 * 2 * cos(1.5708) = 0.0

  ASSERT_NEAR(F(0, 0), 1.0, epsilon);
  ASSERT_NEAR(F(1, 1), 1.0, epsilon);
  ASSERT_NEAR(F(2, 2), 1.0, epsilon);
  ASSERT_NEAR(F(0, 2), expected_F_0_2, epsilon);
  ASSERT_NEAR(F(1, 2), expected_F_1_2, epsilon);
}

TEST_F(UnicycleModelTest, ComputeJacobian_TurningMotion) {
  StateVector X;
  X << 1.0, 2.0, M_PI / 2.0; // theta = 1.5708

  ControlInput U;
  U << 10.0, 1.0; // v = 10, omega = 1

  StateMatrix F = model.computeFt(X, U, dt);

  double v_k = U(0);
  double omega_k = U(1);
  double turn_radius = v_k / omega_k;
  double theta_k = X(2);
  double theta_plus_omega_dt = theta_k + omega_k * dt;

  double expected_F_0_2 =
      turn_radius * (std::cos(theta_plus_omega_dt) - std::cos(theta_k));
  double expected_F_1_2 =
      turn_radius * (std::sin(theta_plus_omega_dt) - std::sin(theta_k));

  ASSERT_NEAR(F(0, 0), 1.0, epsilon);
  ASSERT_NEAR(F(1, 1), 1.0, epsilon);
  ASSERT_NEAR(F(2, 2), 1.0, epsilon);
  ASSERT_NEAR(F(0, 2), expected_F_0_2, epsilon);
  ASSERT_NEAR(F(1, 2), expected_F_1_2, epsilon);
}

// Parametrized test for getNextState
struct GetNextStateTestParams {
  StateVector initial_state;
  ControlInput control_input;
  double dt;
  StateVector expected_final_state;
  std::string test_name;
};

class UnicycleGetNextStateTest
    : public testing::TestWithParam<GetNextStateTestParams> {
protected:
  models::Unicycle<StateSize, ControlSize> model;
};

TEST_P(UnicycleGetNextStateTest, ComputesCorrectNextState) {
  GetNextStateTestParams params = GetParam();
  StateVector result =
      model.getNextState(params.initial_state, params.control_input, params.dt);

  double epsilon = 1e-4;
  EXPECT_NEAR(result(0), params.expected_final_state(0), epsilon)
      << "X mismatch for " << params.test_name;

  EXPECT_NEAR(result(1), params.expected_final_state(1), epsilon)
      << "Y mismatch for " << params.test_name;

  EXPECT_NEAR(result(2), common::normalizeAngle(params.expected_final_state(2)),
              epsilon)
      << "Theta mismatch for " << params.test_name;
}

INSTANTIATE_TEST_SUITE_P(
    UnicycleKinematicsTests, UnicycleGetNextStateTest,
    testing::Values(
        // Test Case 1: Straight Motion (v=10, omega=0, dt=5)
        GetNextStateTestParams{
            {0.0, 0.0, 0.0},  // Initial State: x=0, y=0, theta=0 deg
            {10.0, 0.0},      // Control: v=10 m/s, omega=0 rad/s (0 deg/s)
            5.0,              // dt = 5 seconds
            {50.0, 0.0, 0.0}, // Expected Final State: x=50, y=0, theta=0 deg
            "StraightMotion"},

        // Test Case 2: Rotate In Place (v=0, omega=PI/4, dt=2)
        GetNextStateTestParams{
            {0.0, 0.0, 0.0},   // Initial State: x=0, y=0, theta=0 deg
            {0.0, M_PI / 4.0}, // Control: v=0 m/s, omega=PI/4 rad/s (45 deg/s)
            2.0,               // dt = 2 seconds
            {0.0, 0.0, M_PI / 2.0}, // Expected Final State: x=0, y=0,
                                    // theta=PI/2 rad (90 deg)
            "RotateInPlace"},

        // Test Case 3: Simple Turning (180 Degree Turn)
        GetNextStateTestParams{
            {0.0, 0.0, 0.0}, // Initial State: x=0, y=0, theta=0 deg
            {1.0, 0.1},      // Control: v=1 m/s, omega=0.1 rad/s
            M_PI /
                0.1, // dt = PI/0.1 seconds (approx 31.4159s for 180 deg turn)
            {0.0, 2.0 * (1.0 / 0.1),
             M_PI}, // Expected Final State: x=0, y=2*R, theta=PI rad (180 deg)
            "SimpleTurning_180Deg"},

        // Test Case 4: Angle Wrap Around (Negative Start)
        GetNextStateTestParams{
            {0.0, 0.0,
             -3.0 * M_PI /
                 4.0}, // Initial State: x=0, y=0, theta=-3PI/4 rad (-135 deg)
            {0.0, M_PI / 2.0}, // Control: v=0 m/s, omega=PI/2 rad/s (90 deg/s)
            1.0,               // dt = 1 second
            {0.0, 0.0, -M_PI / 4.0}, // Expected Final State: x=0, y=0,
                                     // theta=-PI/4 rad (-45 deg)
            "AngleWrapAround_NegStart"},

        // Test Case 5: General Turning Motion (Complex Calculation)
        GetNextStateTestParams{
            {10.0, 8.0,
             M_PI}, // Initial State: x=10, y=8, theta=PI rad (180 deg)
            {30.0, 3.3161227778}, // Control: v=30 m/s, omega=3.3161227778 rad/s
                                  // (approx 190 deg/s)
            2.0,                  // dt = 2 seconds
            {6.905826317, 7.454409934,
             -2.79252}, // Expected Final State (in radians, -2.79252 rad ~ -160
                        // deg)
            "GeneralTurningMotion"}),
    [](const testing::TestParamInfo<GetNextStateTestParams> &info) {
      return info.param.test_name;
    });