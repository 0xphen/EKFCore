#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <stdexcept>

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
 * the statistical properties defined by 'covarianceMatrix'.
 *
 * @param covarianceMatrix The desired covariance matrix for the output vector.
 * This matrix must be symmetric and positive-definite.
 * @return An Eigen::Vector of dimension M x 1 with a mean of zero and the
 * specified covariance.
 */
template <int M>
Eigen::Matrix<double, M, 1>
generateCorrelatedNoise(const Eigen::Matrix<double, M, M> &covarianceMatrix) {
  using VectorM = Eigen::Matrix<double, M, 1>;
  using MatrixM = Eigen::Matrix<double, M, M>;

  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0.0, 1.0);

  VectorM w;
  for (int i = 0; i < M; ++i) {
    w(i) = distribution(generator);
  }

  Eigen::LLT<MatrixM> lltOfR(covarianceMatrix);

  if (lltOfR.info() != Eigen::Success) {
    throw std::runtime_error("Covariance matrix is not positive-definite. "
                             "Check your Q and R matrices.");
  }

  MatrixM L = lltOfR.matrixL();

  return L * w;
}
} // namespace common