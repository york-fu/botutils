/**
 * @file lowpass_filter.cc
 * @author York Fu (york-fu@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(int order, double sample_rate, double cutoff_frequency)
    : order_(order), sample_rate_(sample_rate), cutoff_frequency_(cutoff_frequency)
{
  a_coeffs_.resize(order + 1);
  b_coeffs_.resize(order + 1);
  input_buffer_.resize(order + 1);
  output_buffer_.resize(order + 1);

  double wc = 2.0 * M_PI * cutoff_frequency_ / sample_rate_;
  double alpha = sin(wc) / (2.0 * order_);
  double beta = 0.5 * ((1.0 - cos(wc)) / (1.0 + cos(wc)));
  double gamma = (0.5 + beta - alpha) / 2.0;

  for (int i = 0; i <= order_; i++)
  {
    if (i == 0)
    {
      a_coeffs_[i] = gamma;
      b_coeffs_[i] = 2.0 * gamma;
    }
    else
    {
      double sin_term = sin(i * wc) * alpha / (i * sin(alpha));
      double cos_term = cos(i * wc) * beta / cos(beta);
      a_coeffs_[i] = sin_term - cos_term;
      b_coeffs_[i] = 2.0 * gamma * cos(i * wc);
    }
  }
}

double LowPassFilter::filter(double input)
{
  double output = 0.0;

  input_buffer_.insert(input_buffer_.begin(), input);
  input_buffer_.pop_back();

  for (int i = 0; i <= order_; i++)
  {
    output += b_coeffs_[i] * input_buffer_[i];
    if (i > 0)
    {
      output -= a_coeffs_[i] * output_buffer_[i - 1];
    }
  }

  output_buffer_.insert(output_buffer_.begin(), output);
  output_buffer_.pop_back();

  return output;
}
