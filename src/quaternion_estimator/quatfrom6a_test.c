#include <gtest/gtest.h>
#include "quatfrom6a.h"

TEST(QuaternionTest, DoesNotQuatUpdateWithZeroAngularVelocity)
{
  quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
  vector3f w = {0.0f, 0.0f, 0.0f};
  vector3f a = {0.0f, 0.0f, -9.81f};
  float dt = 0.01f;
  float kp = 1.0f;
  float ki = 0.0f;
  vector3f ei = {0.0f, 0.0f, 0.0f};

  // Update the quaternion
  int16_t result = quat_update(&q, w, a, dt, kp, ki, &ei);

  // Check that the function returns -1 to indicate failure
  EXPECT_EQ(result, -1);

  // Check that the quaternion has not been updated
  EXPECT_EQ(q.w, 1.0f);
  EXPECT_EQ(q.x, 0.0f);
  EXPECT_EQ(q.y, 0.0f);
  EXPECT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, QuatUpdateWithNonzeroInputs)
{
  quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
  vector3f w = {0.1f, 0.2f, 0.3f};
  vector3f a = {0.0f, 0.0f, -9.81f};
  float dt = 0.01f;
  float kp = 1.0f;
  float ki = 0.01f;
  vector3f ei = {0.0f, 0.0f, 0.0f};

  // Update the quaternion
  quat_update(&q, w, a, dt, kp, ki, &ei);

  // Check that the quaternion has been updated
  EXPECT_NE(q.w, 1.0f);
  EXPECT_NE(q.x, 0.0f);
  EXPECT_NE(q.y, 0.0f);
  EXPECT_NE(q.z, 0.0f);
}

TEST(QuaternionTest, Quat2EulerReturnsCorrectRollPitchYaw)
{
  quaternion q = {0.998f, 0.000f, 0.052f, 0.000f};

  vector3f rpy = quat2euler(q);
  DOUBLES_EQUAL(0.0, rpy.x, 0.01);
  DOUBLES_EQUAL(0.1, rpy.y, 0.01);
  DOUBLES_EQUAL(0.0, rpy.z, 0.01);
}

TEST(QuaternionTest, Euler2QuatReturnsCorrectQuaternion)
{
  vector3f rpy;
  rpy.x = 0.2; // roll
  rpy.y = 0.3; // pitch
  rpy.z = 0.4; // yaw

  quaternion expected_q;
  expected_q.w = 0.926181;
  expected_q.x = 0.130526;
  expected_q.y = 0.240008;
  expected_q.z = 0.270446;

  quaternion result = euler2quat(rpy);
  float tol = 0.0001;
  EXPECT_NEAR(expected_q.w, result.w, tol);
  EXPECT_NEAR(expected_q.x, result.x, tol);
  EXPECT_NEAR(expected_q.y, result.y, tol);
  EXPECT_NEAR(expected_q.z, result.z, tol);
}