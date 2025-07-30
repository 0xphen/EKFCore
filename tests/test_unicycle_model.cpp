#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "models/UnicycleModel.hpp"

TEST(UnicycleModelTest, GetDimensionsReturnsCorrectValue) {
  models::UnicycleModel model;
  ASSERT_EQ(model.getStateDim(), 3);
  ASSERT_EQ(model.getInputDimension(), 2);
}