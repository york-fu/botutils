#include <iostream>
#include <chrono>
#include <thread>
#include "timer.h"
#include "performance_measurement.h"

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
  Rate loop_rete(1e-3);
  while (true)
  {
    compute_pi(1e5);
    loop_rete.sleep();
  }
}

void job2()
{
  Rate loop_rete(1e-3);
  PerformanceMeasurement pm;
  pm.start();
  while (true)
  {
    compute_pi(1e5);
    loop_rete.sleep();
    pm.stop();
    pm.start();
    pm.print();
  }
}

int main(int argv, char **argc)
{
  uint32_t num_cores = std::thread::hardware_concurrency();
  std::cout << "Number of CPU cores: " << num_cores << std::endl;

  std::vector<std::thread> threads;
  for (int i = 0; i < num_cores - 1; i++)
  {
    threads.push_back(std::thread(job1));
  }
  threads.push_back(std::thread(job2));

  std::thread::native_handle_type handle = threads[num_cores - 1].native_handle();
  int policy = SCHED_FIFO;
  struct sched_param params;
  params.sched_priority = sched_get_priority_max(policy) - 10;
  int ret = 0;
  ret = pthread_setschedparam(handle, policy, &params);
  if (ret)
  {
    std::cout << "Failed to setschedparam\n";
  }

  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(0, &mask);
  ret = pthread_setaffinity_np(handle, sizeof(mask), &mask);
  if (ret)
  {
    std::cout << "Failed to setaffinity\n";
  }

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  for (auto &t : threads)
  {
    t.join();
  }

  return 0;
}