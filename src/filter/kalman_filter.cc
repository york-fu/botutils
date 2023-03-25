/**
 * @file kalman_filter.cc
 * @author York Fu (york-fu@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "kalman_filter.h"

KalmanFilter::KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                           const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q,
                           const Eigen::MatrixXd &R, const Eigen::VectorXd &x0)
    : A_(A), B_(B), H_(H), Q_(Q), R_(R), P_(Eigen::MatrixXd::Identity(A_.rows(), A_.cols())), x_(x0)
{
}

KalmanFilter::KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                           const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q,
                           const Eigen::MatrixXd &R, const Eigen::MatrixXd &P0, const Eigen::VectorXd &x0)
    : A_(A), B_(B), H_(H), Q_(Q), R_(R), P_(P0), x_(x0)
{
}

KalmanFilter::KalmanFilter(int num_states, int num_inputs)
{
  A_ = Eigen::MatrixXd::Identity(num_states, num_states);
  B_ = Eigen::MatrixXd::Zero(num_states, num_inputs);
  H_ = Eigen::MatrixXd::Identity(num_inputs, num_states);
  P_ = Eigen::MatrixXd::Identity(num_states, num_states);
  Q_ = Eigen::MatrixXd::Identity(num_states, num_states);
  R_ = Eigen::MatrixXd::Identity(num_inputs, num_inputs);
  x_ = Eigen::VectorXd::Zero(num_states);
}

void KalmanFilter::setA(const Eigen::MatrixXd &A)
{
  A_ = A;
}

void KalmanFilter::setB(const Eigen::MatrixXd &B)
{
  B_ = B;
}

void KalmanFilter::setH(const Eigen::MatrixXd &H)
{
  H_ = H;
}

void KalmanFilter::setP(const Eigen::MatrixXd &P)
{
  P_ = P;
}

void KalmanFilter::setQ(const Eigen::MatrixXd &Q)
{
  Q_ = Q;
}

void KalmanFilter::setR(const Eigen::MatrixXd &R)
{
  R_ = R;
}

void KalmanFilter::setState(const Eigen::VectorXd &x0)
{
  x_ = x0;
}

Eigen::VectorXd KalmanFilter::getState() const
{
  return x_;
}

Eigen::VectorXd KalmanFilter::predict(const Eigen::VectorXd &u)
{
  x_ = A_ * x_ + B_ * u;
  P_ = A_ * P_ * A_.transpose() + Q_;
  return x_;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::VectorXd &z)
{
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * (z - H_ * x_);
  P_ = (Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K * H_) * P_;
  return x_;
}
