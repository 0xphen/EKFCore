#pragma once

#include <Eigen/Dense>

namespace common {

// State vector: x, y, theta
using StateVector = Eigen::Vector3d;

using ControlInput = Eigen::VectorXd;
} // namespace common
