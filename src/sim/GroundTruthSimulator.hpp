#pragma once

#include <Eigen/Dense>
#include <memory>

#include "common/INoiseGenerator.hpp"
#include "models/IVehicle.hpp"

namespace sim {
/**
 * @brief Simulates the true state evolution of a vehicle for EKF validation.
 *
 * This class generates and maintains the vehicle's true (noisy) state and its
 * ideal (noise-free) path, providing a benchmark for the filter's performance.
 *
 * @tparam StateSize The dimension of the state vector.
 * @tparam ControlSize The dimension of the control input vector.
 */
template <int StateSize, int ControlSize> class GroundTruthSimulator {
public:
  using StateVector =
      typename common::VehicleTypes<StateSize, ControlSize>::StateVector;
  using ControlInput =
      typename common::VehicleTypes<StateSize, ControlSize>::ControlVector;
  using ProcessNoiseMatrix =
      typename common::VehicleTypes<StateSize, StateSize>::StateMatrix;

  using NoiseGeneratorType = common::INoiseGenerator<StateSize>;

  /**
   * @brief Constructs a GroundTruthSimulator instance.
   *
   * @param model A unique_ptr to the concrete vehicle motion model.
   * @param initial_state The initial true state of the vehicle.
   * @param Q The process noise covariance matrix.
   * @param noise_generator A noise generator implementation.
   */
  GroundTruthSimulator(
      std::unique_ptr<models::IVehicle<StateSize, ControlSize>> model,
      const StateVector &initial_state, const ProcessNoiseMatrix &Q,
      const NoiseGeneratorType &noise_generator);

  /**
   * @brief Returns the ideal, noise-free state.
   */
  const StateVector &getPerfectState() const;

  /**
   * @brief Returns the current true state, including process noise.
   */
  const StateVector &getNoisyState() const;

  /**
   * @brief Advances the vehicle's state by one time step.
   *
   * This method applies the motion model and adds process noise to the state.
   *
   * @param control_input The control inputs for this step.
   * @param dt The duration of the time step.
   */
  void advanceState(const ControlInput &control_input, double dt);

private:
  /**
   * @brief The concrete IVehicleModel implementation.
   */
  std::unique_ptr<models::IVehicle<StateSize, ControlSize>> model_;

  /**
   * @brief The current true state, including process noise.
   */
  StateVector x_noisy;

  /**
   * @brief The ideal, noise-free state used for validation.
   */
  StateVector x_true;

  /**
   * @brief The process noise covariance matrix, Q.
   */
  ProcessNoiseMatrix Q_;

  /**
   * @brief The injected noise generator dependency.
   */
  const NoiseGeneratorType &noise_generator_;
};
} // namespace sim