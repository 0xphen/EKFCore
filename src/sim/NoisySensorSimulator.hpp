// #pragma once

// #include <Eigen/Dense>

// #include "common/EkfTraits.hpp"
// #include "sim/GroundTruthSimulator.hpp"

// namespace sim {
// template <int M, int N> class NoisySensor {
// public:
//   using Traits = common::EKFTraits<M, N>;

//   using MeasurementMatrix = typename Traits::MeasurementMatrix;
//   using CovarianceMatrix = typename Traits::CovarianceMatrix;
//   using MeasurementVector = typename Traits::MeasureVector;

//   NoisySensor(GroundTruthSimulator &groundTruthSim, const CovarianceMatrix &R,
//               const MeasurementMatrix &H);

//   MeasurementVector simulateNoisyMeasurement();

// private:
//   GroundTruthSimulator &groundTruth_;
//   CovarianceMatrix R_;
//   MeasurementMatrix H_;
// };
// } // namespace sim