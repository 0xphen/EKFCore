#pragma once

#include <Eigen/Dense>
#include <memory>

#include "models/IVehicle.hpp"

namespace sim {
/**
 * @brief Simulates the true, noisy state evolution of a 2D vehicle.
 *
 * This class maintains the absolute "ground truth" state of the vehicle
 * over time. It uses a specific IVehicleModel implementation to advance
 * this state based on perfect control inputs and models real-world
 * disturbances with process noise.
 */
template <int StateSize, int ControlSize> class GroundTruthSimulator {
public:
  using StateVector =
      typename common::VehicleTypes<StateSize, ControlSize>::StateVector;

  using ControlInput =
      typename common::VehicleTypes<StateSize, ControlSize>::ControlVector;

  using ProcessNoiseMatrix =
      typename common::VehicleTypes<StateSize, StateSize>::StateMatrix;

  /**
   * @brief Constructs a GroundTruthSimulator instance.
   *
   * @param model A std::unique_ptr to a concrete vehicle motion model (e.g.,
   * UnicycleModel). Ownership of the model is transferred to this simulator.
   * @param initial_state The initial true state of the vehicle [x, y, theta] in
   * global coordinates.
   * @param Q The process noise covariance matrix that models unpredicted
   * disturbances.
   */
  GroundTruthSimulator(
      std::unique_ptr<models::IVehicle<StateSize, ControlSize>> model,
      const StateVector &initial_state, const ProcessNoiseMatrix &Q);

  /**
   * @brief Returns the current true state of the vehicle.
   *
   * This provides the perfect, noise-free state for comparison with filter
   * estimates and for generating noisy sensor measurements.
   *
   * @return A reference to the vehicle's current true state vector [x, y,
   * theta].
   */
  const StateVector &getPerfectState() const;

  const StateVector &getNoisyState() const;

  /**
   * @brief Advances the vehicle's true state by one time step, including
   * process noise.
   *
   * This method uses the internally held IVehicleModel to calculate the next
   * state and adds a random process noise vector to it, simulating real-world
   * disturbances.
   *
   * @param control_input The true control inputs [v, omega]
   * for this step. These inputs dictate the true motion for the duration 'dt'.
   * @param dt The duration of the time step (delta time) in seconds.
   */
  void advanceState(const ControlInput &control_input, double dt);

private:
  /**
   * @brief A smart pointer to the concrete IVehicleModel implementation.
   * It defines the kinematic rules for the vehicle's motion.
   */
  std::unique_ptr<models::IVehicle<StateSize, ControlSize>> model_;

  // The current true state of the vehicle, including process noise.
  StateVector x_noisy;

  // The ideal, noise-free state used for validation and comparison.
  StateVector x_true;

  /**
   * @brief The process noise covariance matrix, Q.
   * This matrix models the uncertainty and correlations of the unpredicted
   * disturbances that affect the vehicle's motion.
   */
  ProcessNoiseMatrix Q_;
};
} // namespace sim