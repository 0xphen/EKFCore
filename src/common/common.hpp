#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace common {
constexpr double EPSILON = 1e-6; // Threshold for near-zero angular velocity

enum class SensorType { GPS, LIDAR, WHEEL_ODOMETRY, IMU };

// Normalize angle to [-PI, PI)
inline double normalizeAngle(double angle) {
  double MAX_ANGLE = 2.0 * M_PI;
  double normalized_angle = std::fmod(angle, MAX_ANGLE);

  if (normalized_angle >= M_PI) {
    normalized_angle -= MAX_ANGLE;
  }
  if (normalized_angle < -M_PI) {
    normalized_angle += MAX_ANGLE;
  }
  return normalized_angle;
}

/**
 * @brief Generates a zero-mean Gaussian random vector with a specified
 * covariance.
 *
 * This function uses Cholesky decomposition to transform a vector of
 * independent standard random variables into a correlated random vector with
 * the statistical properties defined by 'covarianceMatrix'. It is
 * essential for simulating realistic sensor or process noise.
 *
 * @param covarianceMatrix The desired covariance matrix for the output vector.
 * This matrix must be symmetric and positive-definite.
 * @return An Eigen::Vector of dimension M x 1 with a mean of zero and the
 * specified covariance.
 */
template <int M>
Eigen::Matrix<double, M, 1>
generateCorrelatedNoise(const Eigen::Matrix<double, M, M> &covarianceMatrix);
} // namespace common
