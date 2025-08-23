#pragma once

#include <Eigen/Dense>

namespace common {
// M is the number of sensor measurements (e.g., 3 for IMU)
// N is the number of state variables (e.g., 3 for x, y, theta)
template <int M, int N> 
struct VehicleTypes {
  using MeasurementVector = Eigen::Matrix<double, M, 1>;
  using MeasurementMatrix = Eigen::Matrix<double, M, N>;
  using MeasurementCovarianceMatrix = Eigen::Matrix<double, M, M>;
};
} // namespace common