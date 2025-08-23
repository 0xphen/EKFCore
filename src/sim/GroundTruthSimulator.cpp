#include <Eigen/Dense>
#include <cassert>
#include <memory>

#include "GroundTruthSimulator.hpp"
#include "common/common.hpp"

namespace sim {
template <int StateSize, int ControlSize>
GroundTruthSimulator<StateSize, ControlSize>::GroundTruthSimulator(
    std::unique_ptr<models::IVehicle<StateSize, ControlSize>> model,
    const StateVector &initial_state, const ProcessNoiseMatrix &Q)
    : model_(std::move(model)), noisy_state_(initial_state),
      perfect_state_(initial_state), Q_(Q) {
  if (!model_) {
    throw std::invalid_argument(
        "GroundTruthSimulator requires a valid IVehicleModel.");
  }
}

template <int StateSize, int ControlSize>
const typename GroundTruthSimulator<StateSize, ControlSize>::StateVector &
GroundTruthSimulator<StateSize, ControlSize>::getPerfectState() const {
  return perfect_state_;
}

template <int StateSize, int ControlSize>
const typename GroundTruthSimulator<StateSize, ControlSize>::StateVector &
GroundTruthSimulator<StateSize, ControlSize>::getNoisyState() const {
  return noisy_state_;
}

template <int StateSize, int ControlSize>
void GroundTruthSimulator<StateSize, ControlSize>::advanceState(
    const ControlInput &control_input, double dt) {
  perfect_state_ = model_->getNextState(perfect_state_, control_input, dt);

  StateVector process_noise = common::generateCorrelatedNoise<StateSize>(Q_);

  noisy_state_ = perfect_state_ + process_noise;
}
} // namespace sim