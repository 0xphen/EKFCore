#include "common/INoiseGenerator.hpp"
#include "common/common.hpp"

namespace common {
template <int M> class NoiseGenerator : public common::INoiseGenerator<M> {
public:
  Eigen::Matrix<double, M, 1>
  generate(const Eigen::Matrix<double, M, M> &covariance) const override {
    return common::generateCorrelatedNoise<M>(covariance);
  }
};
} // namespace common