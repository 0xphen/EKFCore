#include <cassert>
#include <cmath>
#include <iostream>

#include "Unicycle.hpp"
#include "common/common.hpp"

namespace models {
template <int StateSize, int ControlSize>
typename Unicycle<StateSize, ControlSize>::StateVector
Unicycle<StateSize, ControlSize>::getNextState(
    const typename Unicycle<StateSize, ControlSize>::StateVector &current_state,
    const typename Unicycle<StateSize, ControlSize>::ControlVector
        &control_input,
    double dt) const {
  static_assert(StateSize == 3, "UnicycleModel requires StateSize == 3.");
  static_assert(ControlSize == 2, "UnicycleModel requires ControlSize == 2.");

  double x_k = current_state(0);
  double y_k = current_state(1);
  double theta_k = current_state(2);

  double v_k = control_input(0);
  double omega_k = control_input(1);

  typename Unicycle<StateSize, ControlSize>::StateVector next_state;

  if (std::abs(omega_k) < common::EPSILON) {
    // --- Moving straight ---
    next_state(0) = x_k + v_k * std::cos(theta_k) * dt;
    next_state(1) = y_k + v_k * std::sin(theta_k) * dt;
    next_state(2) = theta_k;
  } else {
    // --- Turning Motion ---
    double turn_radius = v_k / omega_k;
    double icc_global_x = x_k - turn_radius * std::sin(theta_k);
    double icc_global_y = y_k + turn_radius * std::cos(theta_k);
    double next_global_theta = common::normalizeAngle(theta_k + omega_k * dt);

    next_state(0) = icc_global_x + turn_radius * std::sin(next_global_theta);
    next_state(1) = icc_global_y - turn_radius * std::cos(next_global_theta);
    next_state(2) = next_global_theta;
  }

  return next_state;
}

template <int StateSize, int ControlSize>
typename Unicycle<StateSize, ControlSize>::StateMatrix
Unicycle<StateSize, ControlSize>::computeFt(
    const typename Unicycle<StateSize, ControlSize>::StateVector &current_state,
    const typename Unicycle<StateSize, ControlSize>::ControlVector
        &control_input,
    double dt) const {
  double theta_k = current_state(2);
  double v_k = control_input(0);     // Linear velocity
  double omega_k = control_input(1); // Angular velocity

  StateMatrix F;
  if (std::abs(omega_k) < common::EPSILON) {
    F << 1.0, 0.0, -v_k * dt * std::sin(theta_k), 0.0, 1.0,
        v_k * dt * std::cos(theta_k), 0.0, 0.0, 1.0;
  } else {
    double turn_radius = v_k / omega_k;
    double theta_plus_omega_dt = theta_k + omega_k * dt;

    F << 1.0, 0.0,
        turn_radius * (std::cos(theta_plus_omega_dt) - std::cos(theta_k)), 0.0,
        1.0, turn_radius * (std::sin(theta_plus_omega_dt) - std::sin(theta_k)),
        0.0, 0.0, 1.0;
  }

  return F;
}

template class Unicycle<3, 2>;
} // namespace models
