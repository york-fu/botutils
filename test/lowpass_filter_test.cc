#include <gtest/gtest.h>
#include "lowpass_filter.h"

TEST(LowPassFilterTest, SimpleTest)
{
  LowPassFilter filter(2, 100, 10);

  double input = 0;
  double output = 0;

  for (int i = 0; i < 100; i++)
  {
    input = i < 50 ? 0 : 1;
    output = filter.filter(input);
  }

  ASSERT_NEAR(output, 0.5, 0.1);
}
