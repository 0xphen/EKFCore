#pragma once

#include "IVehicle.hpp"

namespace models {
/**
 * @brief Implements the kinematic unicycle model for 2D vehicle motion.
 *
 * This concrete model uses linear velocity (v) and angular velocity (omega) as
 * its control inputs, suitable for differential drive robots or simplified
 * car-like kinematics. It provides the non-linear motion function and its
 * Jacobian for EKF prediction.
 */
template <int StateSize, int ControlSize>
class Unicycle : public IVehicle<StateSize, ControlSize> {
public:
  using Base = IVehicle<StateSize, ControlSize>;
  using typename Base::ControlVector;
  using typename Base::StateMatrix;
  using typename Base::StateVector;

  /**
   * @brief Predicts the next state using the unicycle motion equations.
   * @copydoc IVehicleModel::getNextState
   */
  StateVector getNextState(const StateVector &current_state,
                           const ControlVector &control_input,
                           double dt) const override;

  /**
   * @brief Computes the state transition Jacobian (Ft) for the unicycle model.
   * @copydoc IVehicleModel::computeFt
   */
  StateMatrix computeFt(const StateVector &current_state,
                        const ControlVector &control_input,
                        double dt) const override;
};
} // namespace models