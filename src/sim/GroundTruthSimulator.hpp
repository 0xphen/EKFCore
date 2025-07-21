#pragma once

#include <Eigen/Dense>
#include <memory>

#include "models/IVehicleModel.hpp"

/**
 * @brief Simulates the true, noise-free state evolution of a 2D vehicle.
 *
 * This class maintains the absolute "ground truth" state of the vehicle
 * over time. It uses a specific IVehicleModel implementation to advance
 * this state based on perfect control inputs, serving as the unobservable
 * reality for validating the Extended Kalman Filter.
 */
class GroundTruthSimulator {
public:
  /**
   * @brief Constructs a GroundTruthSimulator instance.
   *
   * @param model A std::unique_ptr to a concrete vehicle motion model (e.g.,
   * UnicycleModel). Ownership of the model is transferred to this simulator.
   * This allows the simulator to use polymorphic behavior for state
   * advancement.
   * @param initial_state The initial true state of the vehicle [x, y, theta] in
   * global coordinates.
   */
  GroundTruthSimulator(std::unique_ptr<IVehicleModel> model,
                       const Eigen::Vector3d &initial_state);

  /**
   * @brief Returns the current true state of the vehicle.
   *
   * This provides the perfect, noise-free state for comparison with filter
   * estimates and for generating noisy sensor measurements.
   *
   * @return The vehicle's current true state vector [x, y, theta].
   */
  Eigen::Vector3d getTrueState() const;

  /**
   * @brief Advances the vehicle's true state by one time step.
   *
   * This method uses the internally held IVehicleModel to calculate the next
   * true state based on the provided perfect control inputs and the duration of
   * the time step. The calculated state updates the internal 'state_' member
   * variable.
   *
   * @param perfect_control_input The true, perfect control inputs [v, omega]
   * for this step. These inputs dictate the true motion for the duration 'dt'.
   * @param dt The duration of the time step (delta time) in seconds.
   */
  void advanceState(const Eigen::VectorXd &perfect_control_input, double dt);

private:
  /**
   * @brief A smart pointer to the concrete IVehicleModel implementation.
   * It defines the kinematic rules for the vehicle's motion.
   */
  std::unique_ptr<IVehicleModel> model_;

  /**
   * @brief The current true state of the vehicle.
   * This vector stores the absolute, noise-free position (x, y) and orientation
   * (theta) of the vehicle in global coordinates.
   */
  Eigen::Vector3d state_;
};