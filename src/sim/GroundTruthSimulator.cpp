#include <Eigen/Dense>
#include <cassert>
#include <memory>

#include "GroundTruthSimulator.hpp"
#include "common/common.hpp"

namespace sim {
template <int StateSize, int ControlSize>
GroundTruthSimulator<StateSize, ControlSize>::GroundTruthSimulator(
    std::unique_ptr<models::IVehicle<StateSize, ControlSize>> model,
    const StateVector &initial_state, const ProcessNoiseMatrix &Q,
    const NoiseGeneratorType &noise_generator)
    : model_(std::move(model)), x_true(initial_state), x_noisy(initial_state),
      Q_(Q), noise_generator_(noise_generator) {
  if (!model_) {
    throw std::invalid_argument(
        "GroundTruthSimulator requires a valid IVehicleModel.");
  }
}

template <int StateSize, int ControlSize>
const typename GroundTruthSimulator<StateSize, ControlSize>::StateVector &
GroundTruthSimulator<StateSize, ControlSize>::getPerfectState() const {
  return x_true;
}

template <int StateSize, int ControlSize>
const typename GroundTruthSimulator<StateSize, ControlSize>::StateVector &
GroundTruthSimulator<StateSize, ControlSize>::getNoisyState() const {
  return x_noisy;
}

template <int StateSize, int ControlSize>
void GroundTruthSimulator<StateSize, ControlSize>::advanceState(
    const ControlInput &control_input, double dt) {
  // Advance the perfect state using the noise-free motion model.
  x_true = model_->getNextState(x_true, control_input, dt);

  // Generate process noise from the process noise covariance matrix Q_.
  StateVector process_noise = noise_generator_.generate(Q_);
  // StateVector process_noise = common::generateCorrelatedNoise<StateSize>(Q_);

  // Advance the noisy state by adding process noise to the perfect state.
  x_noisy = x_true + process_noise;
}
} // namespace sim