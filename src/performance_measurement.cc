/**
 * @file performance_measurement.cc
 * @author York Fu (york-fu@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "performance_measurement.h"

PerformanceMeasurement::PerformanceMeasurement(uint32_t max_count) : max_count_(max_count)
{
  reset();
}

void PerformanceMeasurement::reset()
{
  max_ = std::chrono::nanoseconds::min();
  min_ = std::chrono::nanoseconds::max();
  cur_ = std::chrono::nanoseconds::zero();
  avg_ = std::chrono::nanoseconds::zero();
  total_ = std::chrono::nanoseconds::zero();
}

void PerformanceMeasurement::start()
{
  t0_ = std::chrono::high_resolution_clock::now();
}

void PerformanceMeasurement::stop()
{
  t1_ = std::chrono::high_resolution_clock::now();
  cur_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t1_ - t0_);
  times_.push(cur_.count());
  total_ += cur_;

  if (times_.size() > max_count_)
  {
    total_ -= std::chrono::nanoseconds(times_.front());
    times_.pop();
  }

  max_ = std::max(max_, cur_);
  min_ = std::min(min_, cur_);
  avg_ = total_ / times_.size();
}

void PerformanceMeasurement::print(std::string prefix, uint32_t ignore_times)
{
  print_count_++;
  if (print_count_ > ignore_times)
  {
    print_count_ = 0;
    std::cout << prefix
              << "Time Min/Max/Cur/Avg: " << min_.count() / 1e3
              << " / " << max_.count() / 1e3
              << " / " << times_.back() / 1e3
              << " / " << avg_.count() / 1e3
              << std::endl;
  }
}

std::chrono::nanoseconds PerformanceMeasurement::max() const
{
  return max_;
}

std::chrono::nanoseconds PerformanceMeasurement::min() const
{
  return min_;
}

std::chrono::nanoseconds PerformanceMeasurement::cur() const
{
  return cur_;
}

std::chrono::nanoseconds PerformanceMeasurement::avg() const
{
  return avg_;
}
