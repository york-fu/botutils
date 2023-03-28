#include <gtest/gtest.h>
#include "quaternion_estimator.h"

TEST(QuaternionEstimatorTest, UpdateTest)
{
  QuaternionEstimator estimator;
  Eigen::Vector3d w(0.0, 0.0, 0.0);
  Eigen::Vector3d a(0.0, 0.0, 9.81);
  Eigen::Vector3d h(1.0, 0.0, 0.0);
  Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
  double dt = 0.01;
  Eigen::Quaterniond result = estimator.update(w, a, h, q, dt);
  Eigen::Quaterniond expected_result(1.0, 0.0, 0.0, 0.0);
  double tolerance = 1e-9;
  ASSERT_NEAR(expected_result.w(), result.w(), tolerance);
  ASSERT_NEAR(expected_result.x(), result.x(), tolerance);
  ASSERT_NEAR(expected_result.y(), result.y(), tolerance);
  ASSERT_NEAR(expected_result.z(), result.z(), tolerance);
}