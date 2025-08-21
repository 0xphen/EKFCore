#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace common {

using StateVector = Eigen::Vector3d;
using ControlInput = Eigen::Vector2d;
using StateMatrix = Eigen::Matrix3d;

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
 * @brief Generates a random vector with a specified covariance matrix.
 *
 * This function generates a zero-mean Gaussian random vector with a covariance
 * matrix equal to the input 'covarianceMatrix'. It uses Cholesky decomposition
 * to transform a vector of independent random numbers into a correlated vector.
 *
 * @param covarianceMatrix The desired covariance matrix for the output vector.
 * Must be a symmetric, positive-definite matrix.
 * @return A random StateVector with the specified covariance.
 */
StateVector generateCorrelatedNoise(const StateMatrix &covarianceMatrix);
} // namespace common
