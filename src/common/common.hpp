#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace common {

using StateVector = Eigen::Vector3d;
using ControlInput = Eigen::VectorXd;

enum class SensorType { GPS, LIDAR, WHEEL_ODOMETRY };

// Normalize angle to [-PI, PI)
inline double normalizeAngle(double angle) {
  double MAX_ANGLE = 2.0 * M_PI;
  double normalized_angle = std::fmod(angle, MAX_ANGLE);

  if (normalized_angle >= M_PI) {
    normalized_angle -= MAX_ANGLE;
  }
  if (normalized_angle < -M_PI) {
    normalized_angle += MAX_ANGLE;
  }
  return normalized_angle;
}
} // namespace common
