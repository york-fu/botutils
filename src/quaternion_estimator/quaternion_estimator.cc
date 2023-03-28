/**
 * @file quaternion_estimator.cc
 * @author York Fu (york-fu@outlook.com)
 * @brief
 * @version 0.1
 * @date 2023-03-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "quaternion_estimator.h"

QuaternionEstimator::QuaternionEstimator()
{
}

Eigen::Quaterniond QuaternionEstimator::update(const Eigen::Vector3d w, const Eigen::Vector3d a, const Eigen::Vector3d h, Eigen::Quaterniond &q, double dt)
{
  // Normalize the input vectors
  Eigen::Vector3d wn = w.normalized();
  Eigen::Vector3d an = a.normalized();
  Eigen::Vector3d hn = h.normalized();

  // Compute the rotation matrix from the gyroscope measurements
  Eigen::Matrix3d Rg = Eigen::AngleAxisd(wn.norm() * dt, wn).toRotationMatrix();

  // Compute the acceleration vector in the world frame
  Eigen::Vector3d aw = Rg * an;

  // Compute the magnetic field vector in the world frame
  Eigen::Vector3d hw = Rg * hn;

  // Compute the reference vectors for the accelerometer and magnetometer measurements
  Eigen::Vector3d a_ref(0.0, 0.0, -1.0);
  Eigen::Vector3d h_ref(hw.norm(), 0.0, 0.0);

  // Compute the quaternion correction based on the accelerometer and magnetometer measurements
  Eigen::Vector3d v1 = a_ref.cross(aw).normalized();
  Eigen::Vector3d v2 = h_ref.cross(hw).normalized();
  double angle = acos(v1.dot(v2));
  Eigen::Vector3d axis = v1.cross(v2).normalized();
  Eigen::Quaterniond dq(cos(angle / 2.0), sin(angle / 2.0) * axis.x(), sin(angle / 2.0) * axis.y(), sin(angle / 2.0) * axis.z());

  // Update the current quaternion estimate
  q = dq * q;
  q.normalize();

  // Return the updated quaternion
  return q;
}
