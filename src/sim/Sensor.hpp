#pragma once

#include "Eigen/Dense"

#include "common/EkfTraits.hpp"
#include "common/INoiseGenerator.hpp"

namespace sensor {
/**
 * @brief Represents a generic sensor model for state estimation.
 *
 * Defines the parameters and core functionality for simulating a sensor,
 * including its noise profile (R) and its measurement matrix (H).
 *
 * @tparam MeasurementSize The dimension of the sensor measurement vector.
 * @tparam StateSize       The dimension of the vehicle state vector.
 */
template <int MeasurementSize, int StateSize> class Sensor {
public:
  using Traits = common::SensorTypes<MeasurementSize, StateSize>;

  using MeasurementVector = typename Traits::MeasurementVector;
  using MeasurementMatrix = typename Traits::MeasurementMatrix;
  using CovarianceMatrix = typename Traits::CovarianceMatrix;

  using NoiseGeneratorType = common::INoiseGenerator<MeasurementSize>;

  /**
   * @brief Constructs a generic Sensor object.
   *
   * @param R The measurement noise covariance matrix.
   * @param H The measurement matrix that maps state to measurement space.
   * @param noise_generator The noise generator implementation used to simulate
   * noise.
   */
  Sensor(const CovarianceMatrix &R, const MeasurementMatrix &H,
         const NoiseGeneratorType &noise_generator);

  /**
   * @brief Returns a noisy measurement from the sensor.
   *
   * @param true_state The true, noise-free state of the vehicle.
   * @return A noisy measurement vector.
   */
  MeasurementVector
  getMeasurement(const Eigen::Matrix<double, StateSize, 1> &true_state) const;

  const MeasurementMatrix &getMeasurementMatrix() const;

  const CovarianceMatrix &getCovarianceMatrix() const;

private:
  /**
   * @brief Measurement noise covariance matrix, R.
   */
  CovarianceMatrix R_;
  /**
   * @brief Measurement matrix (Jacobian), H.
   */
  MeasurementMatrix H_;

  NoiseGeneratorType &noise_generator_;
};
} // namespace sensor