#pragma once

#include <Eigen/Dense>

#include "../types.hpp"
#include "IVehicleModel.hpp"

namespace models {
/**
 * @brief Implements the kinematic unicycle model for 2D vehicle motion.
 *
 * This concrete model uses linear velocity (v) and angular velocity (omega) as
 * its control inputs, suitable for differential drive robots or simplified
 * car-like kinematics. It provides the non-linear motion function and its
 * Jacobian for EKF prediction.
 */
class UnicycleModel : public models::IVehicleModel {
public:
  /**
   * @brief Returns the dimension of the state vector.
   * @return Always 3 for the unicycle model (x, y, theta).
   * The state of a 2D unicycle model is typically defined by 3 parameters:
   * 1. x-position: The robot's position along the global X-axis.
   * 2. y-position: The robot's position along the global Y-axis.
   * 3. theta (orientation/heading): The robot's angular orientation relative to
   * the global X-axis. These three values fully describe the robot's pose in a
   * 2D environment.
   * @copydoc sim::IVehicleModel::getStateDim
   */
  int getStateDim() const override;

  /**
   * @brief Returns the dimension of the control input vector.
   * @return Always 2 for the unicycle model (linear_velocity,
   * angular_velocity). The control inputs that drive a unicycle model are
   * typically 2 parameters:
   * 1. Linear velocity (v): The speed at which the robot moves forward or
   * backward.
   * 2. Angular velocity (omega): The rate at which the robot changes its
   * heading (turns). These two inputs are sufficient to control the robot's
   * motion in a 2D plane.
   * @copydoc sim::IVehicleModel::getInputDimension
   */
  int getInputDimension() const override;

  /**
   * @brief Predicts the next state using the unicycle motion equations.
   * Expects control_input to be a 2D vector [linear_velocity,
   * angular_velocity].
   * @copydoc sim::IVehicleModel::getNextState
   */
  StateVector getNextState(const StateVector &current_state,
                           const ControlInput &control_input,
                           double dt) const override;

  // /**
  //  * @brief Computes the state transition Jacobian (Ft) for the unicycle
  //  model.
  //  * Expects control_input to be a 2D vector [linear_velocity,
  //  * angular_velocity].
  //  * @copydoc sim::IVehicleModel::computeFt
  //  */
  // Eigen::Matrix3d computeFt(const Eigen::Vector3d &current_state,
  //                           const Eigen::VectorXd &control_input,
  //                           double dt) const override;
};
} // namespace models
