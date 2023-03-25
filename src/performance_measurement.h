/**
 * @file performance_measurement.h
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
#include <queue>
#include <chrono>

class PerformanceMeasurement
{
public:
  PerformanceMeasurement(uint32_t max_count = 1e4);
  void reset();
  void start();
  void stop();
  void print(std::string prefix = "", uint32_t ignore_times = 1000);

  std::chrono::nanoseconds max() const;
  std::chrono::nanoseconds min() const;
  std::chrono::nanoseconds cur() const;
  std::chrono::nanoseconds avg() const;

private:
  std::chrono::high_resolution_clock::time_point t0_;
  std::chrono::high_resolution_clock::time_point t1_;
  std::chrono::nanoseconds max_;
  std::chrono::nanoseconds min_;
  std::chrono::nanoseconds cur_;
  std::chrono::nanoseconds avg_;
  std::chrono::nanoseconds total_;
  std::queue<long long> times_;
  uint32_t max_count_;
  uint32_t print_count_;
};
