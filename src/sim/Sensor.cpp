#include "Sensor.hpp"
#include <Eigen/Dense>

#include "common/common.hpp"

namespace sim {
template <int MeasurementSize, int StateSize>
Sensor<MeasurementSize, StateSize>::Sensor(
    const typename Sensor<MeasurementSize, StateSize>::CovarianceMatrix &R,
    const typename Sensor<MeasurementSize, StateSize>::MeasurementMatrix &H,
    const NoiseGeneratorType &noise_generator)
    : R_(R), H_(H), noise_generator_(noise_generator) {}

template <int MeasurementSize, int StateSize>
typename Sensor<MeasurementSize, StateSize>::MeasurementVector
Sensor<MeasurementSize, StateSize>::getMeasurement(
    const Eigen::Matrix<double, StateSize, 1> &true_state) const {
  // Generate a random noise vector with the specified covariance
  MeasurementVector noise = noise_generator_.generate(R_);

  return H_ * true_state +
         noise; // perfect measurement (Hx) plus the generated noise
}

template <int MeasurementSize, int StateSize>
const typename Sensor<MeasurementSize, StateSize>::MeasurementMatrix &
Sensor<MeasurementSize, StateSize>::getMeasurementMatrix() const {
  return H_;
}

template <int MeasurementSize, int StateSize>
const typename Sensor<MeasurementSize, StateSize>::CovarianceMatrix &
Sensor<MeasurementSize, StateSize>::getCovarianceMatrix() const {
  return R_;
}

template class sim::Sensor<2, 3>;
template class sim::Sensor<3, 3>;
} // namespace sim