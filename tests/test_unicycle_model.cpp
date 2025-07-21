#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "models/UnicycleModel.hpp"

TEST(UnicycleModelTest, GetStateDimReturnsCorrectValue) {
  UnicycleModel model;
  ASSERT_EQ(model.getStateDim(), 3);
}