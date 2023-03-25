#include <chrono>
#include <thread>

void loop_cpp()
{
  double dt = 1e-3;
  auto next_time = std::chrono::steady_clock::now();
  while (true)
  {
    // do something ...

    next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
    std::this_thread::sleep_until(next_time);
  }
}

void loop_linux()
{
  double dt = 1e-3;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    // do something ...

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}