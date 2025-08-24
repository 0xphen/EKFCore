// common/INoiseGenerator.hpp
#pragma once

#include <Eigen/Dense>

namespace common {
/**
 * @brief Interface for generating random noise with a specified covariance.
 *
 * This abstract class defines a contract for any noise generator.
 *
 * @tparam VectorSize The dimension of the noise vector to be generated.
 */
template <int VectorSize> class INoiseGenerator {
public:
  virtual ~INoiseGenerator() = default;

  virtual Eigen::Matrix<double, VectorSize, 1>
  generate(const Eigen::Matrix<double, VectorSize, VectorSize> &covariance)
      const = 0;
};
} // namespace common