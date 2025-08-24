#pragma once

#include "gmock/gmock.h"

#include "common/INoiseGenerator.hpp"

namespace testing {
template <int M> class MockNoiseGenerator : public common::INoiseGenerator<M> {
public:
  MOCK_METHOD((Eigen::Matrix<double, M, 1>), generate,
              ((const Eigen::Matrix<double, M, M> &covariance)),
              (const, override));
};
} // namespace testing