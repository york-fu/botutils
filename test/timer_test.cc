#include <gtest/gtest.h>
#include "timer.h"

TEST(TimerTest, BasicTest) {
  int count = 0;
  Timer timer;

  timer.setInterval(std::chrono::milliseconds(10));
  timer.setCallback([&count](){ ++count; });
  timer.start();

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  timer.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  EXPECT_GE(count, 5);
  EXPECT_LE(count, 6);
}
