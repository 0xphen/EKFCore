// #include "NoisySensorSimulator.hpp"

// namespace sim {

// template <int M, int N>
// NoisySensor<M, N>::NoisySensor(GroundTruthSimulator &groundTruthSim,
//                                const CovarianceMatrix &R,
//                                const MeasurementMatrix &H)
//     : groundTruth_(groundTruthSim), R_(R), H_(H) {}

// template <int M, int N>
// typename NoisySensor<M, N>::MeasurementVector
// NoisySensor<M, N>::simulateNoisyMeasurement() {
//   common::StateVector noise = common::generateCorrelatedNoise(R_);
// }
// } // namespace sim