#include <random>

#include "common.hpp"

namespace common {
StateVector generateCorrelatedNoise(const StateMatrix &covarianceMatrix) {
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0.0, 1.0);

  StateVector w;
  for (int i = 0; i < 3; ++i) {
    w(i) = distribution(generator);
  }

  // Check for positive definiteness and decompose.
  Eigen::LLT<StateMatrix> lltOfR(covarianceMatrix);

  if (lltOfR.info)
    StateMatrix L = lltOfR.matrixL();

  return L * w;
}
} // namespace common