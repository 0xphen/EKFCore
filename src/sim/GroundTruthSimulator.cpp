#include <Eigen/Dense>
#include <memory>

#include "sim/GroundTruthSimulator.hpp"

namespace sim {
GroundTruthSimulator::GroundTruthSimulator(
    std::unique_ptr<models::IVehicleModel> model,
    const common::StateVector &initial_state)
    : model_(std::move(model)), state_(initial_state) {
  if (!model_) {
    throw std::invalid_argument(
        "GroundTruthSimulator requires a valid IVehicleModel.");
  }
}

const common::StateVector &GroundTruthSimulator::getTrueState() const {
  return state_;
}

void GroundTruthSimulator::advanceState(
    const common::ControlInput &control_input, double dt) {
  state_ = model_->getNextState(state_, control_input, dt);
}
} // namespace sim