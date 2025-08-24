#include "common/INoiseGenerator.hpp"
#include "sim/Sensor.hpp"
#include "test_helper.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

using namespace testing;

// This test validates that the Sensor class's getMeasurement function
// correctly applies the measurement matrix and adds the expected noise.
TEST(SensorTest, GetMeasurement_ReturnsExpectedNoisyState) {
  static constexpr int measurement_size = 2;
  static constexpr int state_size = 3;

  Eigen::Vector2d expected_noise;
  expected_noise << 0.1, 0.2;

  MockNoiseGenerator<measurement_size> mock_noise_generator;
  EXPECT_CALL(mock_noise_generator, generate(A<const Eigen::Matrix2d &>()))
      .WillOnce(Return(expected_noise));

  Eigen::Matrix2d R_gps;
  R_gps << 0.04, 0.0, 0.0, 0.04;

  Eigen::Matrix<double, 2, 3> H_gps;
  H_gps << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  Eigen::Vector3d state;
  state << 1.0, 2.0, 3.0;

  sim::Sensor<measurement_size, state_size> gps_sensor(R_gps, H_gps,
                                                       mock_noise_generator);

  auto expected_measurement = H_gps * state;
  auto noisy_measurement = expected_measurement + expected_noise;

  auto result = gps_sensor.getMeasurement(state);

  double epsilon = 1e-6;
  ASSERT_NEAR(result(0), noisy_measurement(0), epsilon);
  ASSERT_NEAR(result(1), noisy_measurement(1), epsilon);
}