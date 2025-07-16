#ifndef TIME_H
#define TIME_H

#include <chrono>
#include <string>
#include <iostream>

namespace mppi_ga
{

struct ScopedTimer
{
  using Clock = std::chrono::high_resolution_clock;
  Clock::time_point start;
  std::string msg;
  ScopedTimer(const std::string &msg) : start{Clock::now()}, msg{msg} {}
  ~ScopedTimer()
  {
    std::cout << msg << ": " << std::chrono::duration_cast<std::chrono::microseconds>
                 (Clock::now()-start).count() << " µs" << std::endl;
  }
};
}

#endif // TIME_H
