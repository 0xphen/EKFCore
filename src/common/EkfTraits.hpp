#pragma once

#include <Eigen/Dense>

namespace common {
/**
 * @brief Defines type aliases for a vehicle's state and control components.
 * @tparam StateSize The dimension of the state vector.
 * @tparam ControlSize The dimension of the control input vector.
 */
template <int StateSize, int ControlSize> struct VehicleTypes {
  using StateVector = Eigen::Matrix<double, StateSize, 1>;
  using ControlVector = Eigen::Matrix<double, ControlSize, 1>;
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
} // namespace common