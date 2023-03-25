/**
 * @file kalman_filter.h
 * @author York Fu (york-fu@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <Eigen/Dense>

class KalmanFilter
{
public:
  KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
               const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q,
               const Eigen::MatrixXd &R, const Eigen::VectorXd &x0);

  KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
               const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q,
               const Eigen::MatrixXd &R, const Eigen::MatrixXd &P0, const Eigen::VectorXd &x0);

  KalmanFilter(int num_states, int num_inputs);

  void setA(const Eigen::MatrixXd &A);
  void setB(const Eigen::MatrixXd &B);
  void setH(const Eigen::MatrixXd &H);
  void setP(const Eigen::MatrixXd &P);
  void setQ(const Eigen::MatrixXd &Q);
  void setR(const Eigen::MatrixXd &R);
  void setState(const Eigen::VectorXd &x0);
  Eigen::VectorXd getState() const;
  Eigen::VectorXd predict(const Eigen::VectorXd &u);
  Eigen::VectorXd update(const Eigen::VectorXd &z);

private:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd P_;
  Eigen::VectorXd x_;
};
