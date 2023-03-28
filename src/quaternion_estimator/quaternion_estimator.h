/**
 * @file quaternion_estimator.h
 * @author York Fu (york-fu@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <Eigen/Dense>

class QuaternionEstimator
{
public:
  QuaternionEstimator();
  Eigen::Quaterniond update(const Eigen::Vector3d w, const Eigen::Vector3d a, const Eigen::Vector3d h, Eigen::Quaterniond &q, double dt);
};

