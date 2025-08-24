#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <vector>

#include "common/common.hpp"

// This test validates that the generateCorrelatedNoise function produces
// a random vector with the correct statistical properties: a zero mean
// and the specified covariance matrix.
TEST(NoiseGeneratorTest, GeneratesCorrectStatisticalProperties) {
  Eigen::Matrix3d R_input;
  R_input << 0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0025;

  const int num_runs = 10000;
  std::vector<Eigen::Vector3d> noise_samples;
  noise_samples.reserve(num_runs);

  for (int i = 0; i < num_runs; ++i) {
    noise_samples.push_back(common::generateCorrelatedNoise<3>(R_input));
  }

  // Calculate the mean of the generated noise samples.
  Eigen::Vector3d samples_sum = Eigen::Vector3d::Zero();
  for (const auto &sample : noise_samples) {
    samples_sum += sample;
  }
  Eigen::Vector3d samples_mean = samples_sum / static_cast<double>(num_runs);

  // Assert that the mean is statistically zero.
  double mean_tolerance = 1e-2;
  ASSERT_NEAR(samples_mean(0), 0.0, mean_tolerance);
  ASSERT_NEAR(samples_mean(1), 0.0, mean_tolerance);
  ASSERT_NEAR(samples_mean(2), 0.0, mean_tolerance);

  // Calculate the sample covariance matrix from the generated noise.
  Eigen::Matrix3d covariance_sum = Eigen::Matrix3d::Zero();
  for (const auto& sample: noise_samples) {
    Eigen::Vector3d centered_sample = sample - samples_mean;
    covariance_sum += centered_sample * centered_sample.transpose();
  }

  Eigen::Matrix3d samples_covariance =covariance_sum/static_cast<double>(num_runs - 1);

  // Assert that the calculated covariance matrix matches the input.
  double cov_tolerance = 1e-2;

  // Check diagonal elements (variances)
  ASSERT_NEAR(samples_covariance(0, 0), R_input(0, 0), cov_tolerance);
  ASSERT_NEAR(samples_covariance(1, 1), R_input(1, 1), cov_tolerance);
  ASSERT_NEAR(samples_covariance(2, 2), R_input(2, 2), cov_tolerance);

  // Check off-diagonal elements (covariances)
  // Check upper triangle
  ASSERT_NEAR(samples_covariance(0, 1), R_input(0, 1), cov_tolerance);
  ASSERT_NEAR(samples_covariance(0, 2), R_input(0, 2), cov_tolerance);
  ASSERT_NEAR(samples_covariance(1, 2), R_input(1, 2), cov_tolerance);

  // Ensure the calculated covariance matrix is symmetric.
  ASSERT_EQ(samples_covariance(1, 0), samples_covariance(0, 1));
  ASSERT_EQ(samples_covariance(2, 0), samples_covariance(0, 2));
  ASSERT_EQ(samples_covariance(2, 1), samples_covariance(1, 2));
}

TEST(NoiseGeneratorTest, ThrowsExceptionForNonPositiveDefiniteMatrix) {
  // A non-positive-definite matrix. The negative diagonal element makes
  // the matrix invalid for Cholesky decomposition.
  Eigen::Matrix3d non_positive_definite_matrix;
  non_positive_definite_matrix << 0.04, 0.0, 0.0,
                                  0.0, -0.04, 0.0,
                                  0.0, 0.0, 0.0025;

  ASSERT_THROW(common::generateCorrelatedNoise<3>(non_positive_definite_matrix),
               std::runtime_error);
}