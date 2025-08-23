#include "Sensor.hpp"
#include <Eigen/Dense>

#include "common/common.hpp"

namespace sensor {
template <int MeasurementSize, int StateSize>
Sensor<MeasurementSize, StateSize>::Sensor(
    const typename Sensor<MeasurementSize, StateSize>::CovarianceMatrix &R,
    const typename Sensor<MeasurementSize, StateSize>::MeasurementMatrix &H)
    : R_(R), H_(H) {}

template <int MeasurementSize, int StateSize>
typename Sensor<MeasurementSize, StateSize>::MeasurementVector
Sensor<MeasurementSize, StateSize>::getMeasurement(
    const Eigen::Matrix<double, StateSize, 1> &true_state) const {
  // Generate a random noise vector with the specified covariance
  typename Sensor<MeasurementSize, StateSize>::MeasurementVector noise =
      common::generateCorrelatedNoise<MeasurementSize>(R_);

  return H_ * true_state +
         noise; // perfect measurement (Hx) plus the generated noise

  const typename Sensor<MeasurementSize, StateSize>::MeasurementMatrix
      Sensor<MeasurementSize, StateSize>::getMeasurementMatrix {
    return H_;
  }

  const typename Sensor<MeasurementSize, StateSize>::CovarianceMatrix
      Sensor<MeasurementSize, StateSize>::getCovarianceMatrix {
    return R_;
  }
}
} // namespace sensor