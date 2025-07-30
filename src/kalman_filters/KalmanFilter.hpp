#pragma once

#include "common/common.hpp"

namespace kalman_filters {
class KalmanFilter {
public:
  /**
   * @brief Performs the prediction step of the Kalman Filter.
   * @param control_input The control input vector (u).
   * @param dt The time step duration.
   */
  virtual void predict(common::ControlInput &control_input, double dt);

private:
};
} // namespace kalman_filters