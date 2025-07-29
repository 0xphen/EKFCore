#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "models/UnicycleModel.hpp"

TEST(UnicycleModelTest, GetStateDimReturnsCorrectValue) {
  models::UnicycleModel model;
  ASSERT_EQ(model.getStateDim(), 3);
}