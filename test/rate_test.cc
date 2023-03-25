#include <gtest/gtest.h>
#include "timer.h"

TEST(RateTest, SleepTest)
{
  double expected_period = 1e-2;
  Rate rate(expected_period);

  std::vector<double> actual_periods(10);
  for (int i = 0; i < 10; i++)
  {
    auto start_time = std::chrono::steady_clock::now();
    rate.sleep();
    auto end_time = std::chrono::steady_clock::now();
    double actual_period = std::chrono::duration<double>(end_time - start_time).count();
    actual_periods[i] = actual_period;
  }

  for (auto actual_period : actual_periods)
  {
    EXPECT_NEAR(actual_period, expected_period, 0.005);
  }
}
