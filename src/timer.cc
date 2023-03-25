/**
 * @file timer.cc
 * @author York Fu (york-fu@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "timer.h"

Timer::Timer() : interval_(0), is_running_(false) {}

Timer::~Timer()
{
  stop();
}

void Timer::start()
{
  is_running_ = true;
  thread_ = std::thread(&Timer::run, this);
}

void Timer::stop()
{
  is_running_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void Timer::setInterval(std::chrono::microseconds interval)
{
  interval_ = interval;
}

void Timer::setCallback(std::function<void()> callback)
{
  callback_ = callback;
}

void Timer::run()
{
  start_time_ = std::chrono::high_resolution_clock::now();

  while (is_running_)
  {
    std::this_thread::sleep_until(start_time_ += interval_);
    callback_();
  }
}

Rate::Rate(double dt) : dt_(dt)
{
  t_ = std::chrono::steady_clock::now();
}

Rate::~Rate()
{
}

void Rate::sleep()
{
  t_ += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_));
  std::this_thread::sleep_until(t_);
}
