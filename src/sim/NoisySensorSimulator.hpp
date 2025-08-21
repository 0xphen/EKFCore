#pragma once

#include "sim/GroundTruthSimulator.hpp"

namespace sim {
class NoisySensor {
public:
  NoisySensor(GroundTruthSimulator &groundTruthSim,
              const common::StateMatrix &R, const common::StateMatrix &H);

  common::StateVector simulateMeasurement();

private:
  GroundTruthSimulator &groundTruth_;
  common::StateMatrix R_;
  common::StateMatrix H_;
};
} // namespace sim