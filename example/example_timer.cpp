#include <iostream>
#include <chrono>
#include <thread>
#include "timer.h"
#include "performance_measurement.h"

PerformanceMeasurement pm;

double compute_pi(int num_steps)
{
  double sum = 0.0;
  double step = 1.0 / (double)num_steps;

  for (int i = 0; i < num_steps; i++)
  {
    double x = (i + 0.5) * step;
    sum += 4.0 / (1.0 + x * x);
  }
  return step * sum;
}

void job1()
{
  compute_pi(1e4);
}

void job2()
{
  pm.start();
  compute_pi(1e4);
  pm.stop();
  pm.print();
}

int main(int argv, char **argc)
{
  uint32_t num_cores = std::thread::hardware_concurrency();
  std::cout << "Number of CPU cores: " << num_cores << std::endl;
  double dt = 1e-3;
  Timer timer[num_cores];
  for (uint32_t i = 0; i < num_cores - 1; i++)
  {
    timer[i].setInterval(std::chrono::microseconds((uint32_t)(dt * 1e6)));
    timer[i].setCallback(job1);
    timer[i].start();
  }
  timer[num_cores - 1].setInterval(std::chrono::microseconds((uint32_t)(dt * 1e6)));
  timer[num_cores - 1].setCallback(job2);
  timer[num_cores - 1].start();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  return 0;
}