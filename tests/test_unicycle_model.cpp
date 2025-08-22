#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>

#include "models/UnicycleModel.hpp"
#include "common/common.hpp"

static constexpr int StateSize = 3;
static constexpr int ControlSize = 2;

using StateVector = models::IVehicleModel<StateSize, ControlSize>::StateVector;
using ControlInput = models::IVehicleModel<StateSize, ControlSize>::ControlVector;

TEST(UnicycleModelTest, GetDimensionsReturnsCorrectValue) {
  ASSERT_EQ(static_cast<int>(models::UnicycleModel<StateSize, ControlSize>::StateVector::SizeAtCompileTime), 3);

  ASSERT_EQ(static_cast<int>(models::UnicycleModel<StateSize, ControlSize>::ControlVector::SizeAtCompileTime), 2);
};


struct GetNextStateTestParams {
  StateVector initial_state;
  ControlInput control_input;
  double dt;
  StateVector expected_final_state;
  std::string test_name;
};

class UnicycleModelGetNextStateTest
    : public testing::TestWithParam<GetNextStateTestParams> {
protected:
  models::UnicycleModel<StateSize, ControlSize> model;
};

TEST_P(UnicycleModelGetNextStateTest, ComputesCorrectNextState) {
  GetNextStateTestParams params = GetParam();
  StateVector result =
      model.getNextState(params.initial_state, params.control_input, params.dt);

  std::cout << "X: " << result(0) << std::endl;
  std::cout << "Y: " << result(1) << std::endl;
  std::cout << "Z: " << result(2) << std::endl;

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
    UnicycleModelKinematicsTests, UnicycleModelGetNextStateTest,
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