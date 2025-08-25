#pragma once

#include <Eigen/Dense>

namespace common {
/**
 * @brief Defines type aliases for a robot's state and control components.
 * @tparam StateSize The dimension of the state vector.
 * @tparam ControlSize The dimension of the control input vector.
 */
template <int StateSize, int ControlSize> struct VehicleTypes {
  using StateVector = Eigen::Matrix<double, StateSize, 1>;
  using ControlVector = Eigen::Matrix<double, ControlSize, 1>;
  using ControlInputMatrix = Eigen::Matrix<double, StateSize, ControlSize>;
  using StateMatrix = Eigen::Matrix<double, StateSize, StateSize>;
};

/**
 * @brief Defines type aliases for a sensor's measurements.
 * @tparam MeasurementSize The dimension of the measurement vector.
 * @tparam StateSize The dimension of the state vector.
 */
template <int MeasurementSize, int StateSize> struct SensorTypes {
  using MeasurementVector = Eigen::Matrix<double, MeasurementSize, 1>;
  using MeasurementMatrix = Eigen::Matrix<double, MeasurementSize, StateSize>;
  using CovarianceMatrix =
      Eigen::Matrix<double, MeasurementSize, MeasurementSize>;
};

/**
 * @brief EKF predicted state and covariance (after time update).
 * @tparam N State dimension.
 */
template <int N> struct EkfPrediction {
  // Predicted a priori state vector.
  Eigen::Matrix<double, N, 1> x_predict;

  // Predicted a priori covariance matrix.
  Eigen::Matrix<double, N, N> p_predict;
};

/**
 * @brief EKF corrected state and covariance (after measurement update).
 * @tparam N State dimension.
 */
template <int N> struct EkfCorrection {
  // Updated a posteriori state vector.
  Eigen::Matrix<double, N, 1> x_final;

  // Updated a posteriori covariance matrix.
  Eigen::Matrix<double, N, N> P_final;
};
} // namespace common