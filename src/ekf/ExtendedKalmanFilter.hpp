#pragma once

#include "common/EkfTraits.hpp"
#include "sim/Sensor.hpp"

namespace ekf {
/**
 * @brief A stateless, generic implementation of the Extended Kalman Filter.
 *
 * This class provides the core predict and update functionalities of the EKF,
 * designed to be reusable for any state and measurement size. It is stateless
 * as it does not hold the filter's state or covariance.
 *
 * @tparam StateSize    The dimension of the state vector.
 * @tparam ControlSize  The dimension of the control input vector.
 */
template <int StateSize, int ControlSize> class Ekf {
public:
  using Traits = common::VehicleTypes<StateSize, ControlSize>;

  using StateVector = typename Traits::StateVector;
  using ControlVector = typename Traits::ControlVector;
  using StateMatrix = typename Traits::StateMatrix;
  using ControlInputMatrix = typename Traits::ControlInputMatrix;

  /**
   * @brief Performs the EKF prediction (time update) step.
   *
   * This function projects the current state and its uncertainty forward in
   * time. It uses the system's dynamics and a control input.
   *
   * @param X The a posteriori state estimate from the previous step.
   * @param U The control input vector (e.g., acceleration, steering).
   * @param P The a posteriori state covariance from the previous step.
   * @param B The control-input matrix that maps control inputs to the state.
   * @param Ft The state transition matrix (or Jacobian), which describes how
   * the state evolves.
   * @param Q The process noise covariance matrix, accounting for model
   * uncertainty.
   * @return An EkfPrediction struct containing the a priori predicted state and
   * its covariance.
   */
  common::EkfPrediction<StateSize>
  predict(const StateVector &X, const ControlVector &U, const StateMatrix &P,
          const ControlInputMatrix &B, const StateMatrix &Ft,
          const StateMatrix &Q);

  /**
   * @brief Performs the EKF measurement update step.
   *
   * This function refines the predicted state and its uncertainty
   * using a new measurement from a sensor.
   *
   * @tparam MeasurementSize The dimension of the measurement vector.
   * @param predicted_state The a priori state estimate from the prediction
   * step.
   * @param P The a priori covariance from the prediction step.
   * @param measurement The new measurement from the sensor.
   * @param measurement_noise_R The measurement noise covariance matrix.
   * @param measurement_jacobian_H The measurement Jacobian.
   * @return An EkfCorrection struct containing the a posteriori (corrected)
   * state and covariance.
   */
  template <int MeasurementSize>
  common::EkfCorrection<StateSize>
  update(const StateVector &P, const StateMatrix &predicted_covariance,
         const Eigen::Matrix<double, MeasurementSize, 1> &measurement,
         const Eigen::Matrix<double, MeasurementSize, MeasurementSize>
             &measurement_noise_R,
         const Eigen::Matrix<double, MeasurementSize, StateSize>
             &measurement_jacobian_H);
};
} // namespace ekf