#include "common/common.hpp"

namespace sim {
template <int M> class NoiseGenerator : INoiseGenerator<M> {
public:
  virtual Eigen::Matrix<double, M, 1>
  generate(const Eigen::Matrix<double, M, M> &covariance) override {
    return common::generateCorrelatedNoise<M>(covariance);
  }
};
} // namespace sim