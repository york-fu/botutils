/**
 * @file lowpass_filter.h
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
#include <cmath>
#include <vector>

class LowPassFilter
{
public:
  LowPassFilter(int order, double sample_rate, double cutoff_frequency);
  double filter(double input);

private:
  int order_;
  double sample_rate_;
  double cutoff_frequency_;
  std::vector<double> a_coeffs_;
  std::vector<double> b_coeffs_;
  std::vector<double> input_buffer_;
  std::vector<double> output_buffer_;
};
