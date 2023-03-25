#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "kalman_filter.h"

TEST(KalmanFilterTest, TestPredictAndUpdate)
{
  // Define system model and initial state
  Eigen::MatrixXd A(2, 2);
  A << 1, 1, 0, 1;
  Eigen::MatrixXd B(2, 1);
  B << 0.5, 1;
  Eigen::MatrixXd H(1, 2);
  H << 1, 0;
  Eigen::MatrixXd Q(2, 2);
  Q << 1, 0, 0, 1;
  Eigen::MatrixXd R(1, 1);
  R << 1;
  Eigen::VectorXd x0(2);
  x0 << 0, 0;

  KalmanFilter kf(A, B, H, Q, R, x0);

  // Define input and measurement vectors
  Eigen::VectorXd u(1);
  u << 1;
  Eigen::VectorXd z(1);
  z << 1;

  // Predict and update
  Eigen::VectorXd x_pred = kf.predict(u);
  Eigen::VectorXd x_updated = kf.update(z);

  Eigen::VectorXd x_pred_expected(2);
  x_pred_expected << 0.5, 1;
  Eigen::VectorXd x_updated_expected(2);
  x_updated_expected << 0.875, 1.125;

  ASSERT_TRUE(x_pred.isApprox(x_pred_expected, 1e-4));
  ASSERT_TRUE(x_updated.isApprox(x_updated_expected, 1e-4));
}

TEST(KalmanFilterTest, SingleParticle3D)
{
  // Define the system model
  double dt = 0.001; // time step
  Eigen::MatrixXd A(6, 6);
  A << 1, 0, 0, dt, 0, 0,
      0, 1, 0, 0, dt, 0,
      0, 0, 1, 0, 0, dt,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;

  Eigen::MatrixXd B(6, 3);
  B << dt * dt / 2, 0, 0,
      0, dt * dt / 2, 0,
      0, 0, dt * dt / 2,
      dt, 0, 0,
      0, dt, 0,
      0, 0, dt;

  Eigen::MatrixXd H(6, 6);
  H << 1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;

  Eigen::MatrixXd Q(6, 6);
  double q = 0.1; // process noise
  Q << q * dt * dt * dt / 3, 0, 0, q * dt * dt / 2, 0, 0,
      0, q * dt * dt * dt / 3, 0, 0, q * dt * dt / 2, 0,
      0, 0, q * dt * dt * dt / 3, 0, 0, q * dt * dt / 2,
      q * dt * dt / 2, 0, 0, q * dt, 0, 0,
      0, q * dt * dt / 2, 0, 0, q * dt, 0,
      0, 0, q * dt * dt / 2, 0, 0, q * dt;

  Eigen::MatrixXd R(6, 6);
  double r = 1.0; // measurement noise
  R << r, 0, 0, 0, 0, 0,
      0, r, 0, 0, 0, 0,
      0, 0, r, 0, 0, 0,
      0, 0, 0, r, 0, 0,
      0, 0, 0, 0, r, 0,
      0, 0, 0, 0, 0, r;

  Eigen::VectorXd x0(6);
  x0 << 0, 0, 0, 0, 0, 0;

  KalmanFilter kf(A, B, H, Q, R, x0);

  // Define the true state
  double true_x = 5.0;
  double true_y = -3.0;
  double true_z = 2.0;
  double true_vx = 1.0;
  double true_vy = -2.0;
  double true_vz = 3.0;

  // Simulate the system for 10 seconds
  for (int i = 0; i < 1000; ++i)
  {
    // Generate a noisy measurement
    Eigen::VectorXd z(6);
    z << true_x + r * (2 * drand48() - 1),
        true_y + r * (2 * drand48() - 1),
        true_z + r * (2 * drand48() - 1),
        true_vx + r * (2 * drand48() - 1),
        true_vy + r * (2 * drand48() - 1),
        true_vz + r * (2 * drand48() - 1);

    // Update the Kalman filter
    Eigen::VectorXd u(3);
    u << 0.1, 0.2, -0.1;
    kf.predict(u);
    kf.update(z);

    // Update the true state
    true_x += true_vx * dt + 0.5 * dt * dt * u(0);
    true_y += true_vy * dt + 0.5 * dt * dt * u(1);
    true_z += true_vz * dt + 0.5 * dt * dt * u(2);
    true_vx += u(0) * dt;
    true_vy += u(1) * dt;
    true_vz += u(2) * dt;
  }

  // Check that the estimated state is close to the true state
  Eigen::VectorXd x = kf.getState();
  double eps = 0.1; // tolerance
  EXPECT_NEAR(x(0), true_x, eps);
  EXPECT_NEAR(x(1), true_y, eps);
  EXPECT_NEAR(x(2), true_z, eps);
  EXPECT_NEAR(x(3), true_vx, eps * 3);
  EXPECT_NEAR(x(4), true_vy, eps * 3);
  EXPECT_NEAR(x(5), true_vz, eps * 3);
}