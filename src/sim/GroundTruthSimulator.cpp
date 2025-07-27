#include <Eigen/Dense>
#include <memory>

#include "sim/GroundTruthSimulator.hpp"

namespace sim {
sim::GroundTruthSimulator::GroundTruthSimulator(
    std::unique_ptr<models::IVehicleModel> model,
    const Eigen::Vector3d &initial_state)
    : model_(std::move(model)), state_(initial_state) {
  if (!model_) {
    throw std::invalid_argument(
        "GroundTruthSimulator requires a valid IVehicleModel.");
  }
}

const Eigen::Vector3d &sim::GroundTruthSimulator::getTrueState() const {
  return state_;
}

void sim::GroundTruthSimulator::advanceState(
    const Eigen::VectorXd &control_input, double dt) {
  state_ = model_->getNextState(state_, control_input, dt);
}
} // namespace sim