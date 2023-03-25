/**
 * @file timer.h
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
#include <vector>
#include <functional>
#include <chrono>
#include <thread>

class Timer
{
public:
  Timer();
  ~Timer();
  void start();
  void stop();
  void setInterval(std::chrono::microseconds interval);
  void setCallback(std::function<void()> callback);

private:
  void run();
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::microseconds interval_;
  std::function<void()> callback_;
  std::thread thread_;
  bool is_running_;
};

class Rate
{
public:
  Rate(double dt);
  ~Rate();
  void sleep();

private:
  std::chrono::time_point<std::chrono::steady_clock> t_;
  std::chrono::duration<double> dt_;
};
