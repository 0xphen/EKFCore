#pragma once

#include "Eigen/Dense"

namespace sensor {
/**
 * @brief Represents a generic sensor model for state estimation.
 *
 * This class serves as a container for the parameters that define a sensor's
 * characteristics, including its measurement matrix (H) and its noise
 * profile (R). The templated design allows it to be used for any sensor
 * type with a known number of measurements (M) and state variables (N).
 * @tparam M The dimension of the sensor measurement vector.
 * @tparam N The dimension of the vehicle state vector.
 */
template <int M, int N> 
class Sensor {
public:
  // Public type aliases for clarity and reusability
  using MeasurementVector = Eigen::Matrix<double, M, 1>;
  using MeasurementMatrix = Eigen::Matrix<double, M, N>;
  using CovarianceMatrix = Eigen::Matrix<double, M, M>;

  /**
   * @brief Constructs a generic Sensor object.
   *
   * @param R The measurement noise covariance matrix.
   * @param H The measurement matrix (Jacobian) that maps state to measurement
   * space.
   */
  Sensor(const CovarianceMatrix &R, const MeasurementMatrix &H);

  /**
   * @brief Returns a noisy measurement from the sensor.
   *
   * This is the core functionality of the sensor model. It takes the true
   * state and uses the measurement matrix (H) and noise covariance (R)
   * to simulate a realistic sensor reading.
   *
   * @param true_state The true, noise-free state of the vehicle.
   * @return A noisy measurement vector.
   */
  MeasurementVector
  getMeasurement(const Eigen::Matrix<double, N, 1> &true_state) const;

private:
  /**
   * @brief The measurement noise covariance matrix.
   */
  CovarianceMatrix R_;
  /**
   * @brief The measurement matrix (Jacobian).
   */
  MeasurementMatrix H_;
};
} // namespace models