#include <Eigen/Dense>
#include <cassert>
#include <iostream>
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
  StateVector next_perfect_state =
      model_->getNextState(x_true, control_input, dt);

  StateVector next_state_from_noisy_model =
      model_->getNextState(x_noisy, control_input, dt);

  StateVector process_noise = noise_generator_.generate(Q_);

  x_true = next_perfect_state;
  x_noisy = next_state_from_noisy_model + process_noise;
}

template class sim::GroundTruthSimulator<3, 2>;
} // namespace sim