#include <gtest/gtest.h>
#include <thread>
#include "performance_measurement.h"

TEST(PerformanceMeasurementTest, BasicTest)
{
  PerformanceMeasurement pm;

  for (int i = 0; i < 10; i++)
  {
    pm.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    pm.stop();
    EXPECT_GE(pm.cur(), std::chrono::milliseconds(1));
    EXPECT_LE(pm.cur(), std::chrono::milliseconds(4));
    EXPECT_GE(pm.avg(), std::chrono::milliseconds(1));
    EXPECT_LE(pm.avg(), std::chrono::milliseconds(4));
    EXPECT_GE(pm.max(), pm.min());
  }

  pm.reset();
  EXPECT_EQ(pm.cur().count(), 0);
  EXPECT_EQ(pm.avg().count(), 0);
  EXPECT_LE(pm.max(), pm.min());

  pm.print();
}
